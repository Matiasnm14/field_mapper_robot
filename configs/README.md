# Perfiles de configuración

`simulation_dsv60.json` es el perfil ejecutable de planificación y adquisición
simulada: DSV de 60 mm, paso de 5 mm, BORE de referencia de Ø100 mm por 254 mm
y diez muestras por visita. Sus posiciones, transformación y parámetros de
robot/sensor son sintéticos; no representan una calibración ni valores de
operación del laboratorio. El plan vigente es
[Plan_Codex_M1Pro_DSV_60mm.md](../Plan_Codex_M1Pro_DSV_60mm.md).

`m1pro_lab.json` es una plantilla independiente e incompleta para el **Dobot
M1 Pro confirmado**. Conserva `host=null` y firmware desconocido; no permite
conectarse hasta indicar la IP real. Su esquema y la revisión del SDK están
documentados en [docs/M1PRO_SDK.md](../docs/M1PRO_SDK.md).

## Esquema de planificación y simulación

Se exigen `schema_version=1`, `name`, `mode="simulation"`, `units` y `geometry`.
Las unidades son `mm`, `deg` y `T`. Se rechazan campos desconocidos o faltantes,
claves JSON duplicadas, números no finitos y tipos incompatibles.

| Sección | Parámetros |
|---|---|
| `geometry` | Radio y paso de la malla; radio y longitud del BORE de referencia. `radio/paso` debe ser entero y la esfera debe caber en el BORE. |
| `scan` | Rotación `rotation_magnet_from_scan`, vertical física unitaria `up_base` e indicadores booleanos `pre_reference` y `post_reference`. |
| `acquisition` | Estabilización, muestras por visita, plazos de movimiento/lectura, tolerancias de posición/ángulo, intervalo de llegada estable y máximo de reintentos del sensor. |
| `simulation` | Semilla, pose inicial, transformación, herramienta, modelo escalar de campo, movimiento, intervalo/unidad de muestras y fallos inyectables. |

`plan` admite perfiles sin `acquisition` ni `simulation`, o con ambas secciones
en `null`. `simulate` exige ambas completas. Si una sección está presente,
se valida también al generar un plan.

## Marco de barrido y referencias

La retícula usa índices `(iu, iv, ih)` y coordenadas `(u, v, h)`. `h` es altura
física; el recorrido completa cada capa horizontal y sube sin descender dentro
de la malla. El perfil de referencia convierte `(u, v, h)` a
`(x_imán, y_imán, z_imán) = (h, u, v)`. `z_imán` sigue siendo el eje del BORE.
`path.csv` exporta los índices y las coordenadas de ambos marcos.

`scan.rotation_magnet_from_scan` es una rotación propia 3×3. Si se dispone de
`simulation.T_base_magnet`, se exige que su rotación lleve el eje `+h` a
`scan.up_base`. En el perfil sintético ese vector es `[0, 0, 1]`, por lo que
subir en `h` corresponde a subir en `Z_base`. La transformación y la herramienta
siguen siendo sintéticas; no constituyen calibración física del M1 Pro.

Si se omite `scan`, se usa la base de referencia y ambas referencias activadas.
`scan.pre_reference` mide el centro antes de la malla y `scan.post_reference`
después. Cada indicador puede desactivarse de forma independiente; no se
intercalan referencias entre capas. Las fases `pre_reference`, `grid_scan` y
`post_reference` y sus transiciones quedan registradas por separado. Cada fase
empieza tras la llegada confirmada a su primer punto; durante la entrada se
registra `current_phase="transition"`. Esas transiciones simuladas no validan
las rutas físicas de entrada o retirada.

## Adquisición y modelo sintético

`samples_per_point` es un entero positivo y se aplica también a cada referencia.
`max_sensor_retries` es un entero entre 0 y 10 y cuenta reintentos adicionales
del lote completo. Plazos y tolerancias son positivos; `settle_time_s` y
`stable_time_s` pueden ser cero. Todos los tiempos operativos de la demo
avanzan un reloj virtual compartido, sin espera real equivalente.

`simulation.seed` debe ser un entero no negativo. El modelo escalar es:

```text
B_n [T] = base_field_T
          + sum(gradient_T_per_mm[i] * p[i])
          + sum(quadratic_T_per_mm2[i] * p[i]**2)
          + drift_T_per_s * tiempo_virtual_s
          + ruido_normal(0, noise_std_T)
```

`p` es la posición sensible reconstruida en el marco del imán, en mm.
`sensitive_axis_magnet` es un vector unitario que declara el eje de ese escalar;
el modelo no genera tres componentes del campo. `original_unit` admite `T` o
`G`, conservadas junto al valor normalizado (`1 G = 1e-4 T`).
La semilla reproduce el ruido; los tiempos UTC e identificadores de ejecución
cambian entre corridas.

`T_base_magnet` es una matriz rígida 4×4 y `rotation_base_flange` una rotación
propia 3×3. `offset_flange_sensor_mm` expresa la herramienta y `pose_frame`
selecciona `flange` o `tcp`. La pose inicial se declara mediante
`initial_sensor_position_magnet_mm` y `angle_deg`. En el ejemplo todos estos
valores, la velocidad y las tolerancias corresponden exclusivamente a la demo.

## Fallos opcionales

Los campos de fallos tienen valores predeterminados si se omiten:

| Campo | Interpretación |
|---|---|
| `motion_fault_at_visit` | `null` o índice desde 0, incluyendo referencias; debe existir en la corrida. |
| `motion_fault_mode` | `timeout` por defecto; también `out_of_tolerance`, `connection_loss`, `accepted_only`. |
| `sensor_fault_at_read` | `null` o petición desde 1, dentro de las lecturas previstas antes de fallos. |
| `sensor_fault_mode` | `timeout` por defecto; también `stale`, `invalid`, `unit`, `connection_loss`. |
| `sensor_fault_persistent` | `true` por defecto; `false` inyecta solo en la petición seleccionada. Una pérdida de conexión sigue finalizando la corrida. |

La CLI permite sustituirlos con `--fail-motion-at-visit`, `--motion-fault`,
`--fail-sensor-at-read`, `--sensor-fault` y `--sensor-fault-once`.
Cada ejecución conserva la configuración efectiva en `config_used.json`.

## Variantes locales

Copiar el perfil a `*.local.json`, ignorado por Git, y modificar los parámetros
de simulación necesarios. Con diez muestras por visita y sin reintentos:

| Paso | Puntos de malla | Visitas con ambas referencias | Muestras con ambas referencias | Visitas/muestras sin referencias |
|---|---:|---:|---:|---:|
| 5 mm | 925 | 927 | 9.270 | 925 / 9.250 |
| 10 mm, piloto | 123 | 125 | 1.250 | 123 / 1.230 |
| 2,5 mm | 7.153 | 7.155 | 71.550 | 7.153 / 71.530 |

La retícula previa al recorte se limita a 2.000.000 de candidatos; se rechazan
pasos que excedan ese límite. Para una demo sin referencias, establecer
`scan.pre_reference=false` y `scan.post_reference=false` en la copia local.

## Perfil de laboratorio M1 Pro

```bash
python -m field_mapper inspect-robot --config configs/m1pro_lab.json
python -m field_mapper scan --config configs/m1pro_lab.json --mode hardware
```

Con el perfil distribuido, `inspect-robot` rechaza `host=null` antes de abrir
conexiones. Introducir la IP real en una copia local y verificar, como operador,
el modo TCP/IP secondary development documentado para el controlador. La
versión exacta del programa referido como DobotStudio sigue pendiente; no se
cambia su modo automáticamente. La inspección consulta el robot sin habilitar,
mover, limpiar alarmas, resetear ni ejecutar home. El firmware puede quedar
`null` porque no se presume una consulta de versión no verificada.

`scan --mode hardware` valida los requisitos y falla con la plantilla incompleta.
La adquisición física todavía no se ejecuta: siguen pendientes el sensor real
y las rutas de entrada/retirada verificadas, además de la calibración y los
parámetros de operación. No completar valores desconocidos con los datos
sintéticos del perfil de simulación. El contrato detallado y los límites del
adaptador están en [la documentación M1 Pro](../docs/M1PRO_SDK.md).
