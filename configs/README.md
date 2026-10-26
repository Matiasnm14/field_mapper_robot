# Perfiles de configuración

`simulation_dsv60.json` es el perfil ejecutable de planificación y adquisición
simulada: DSV de 60 mm, paso de 5 mm, BORE de referencia de Ø100 mm por 254 mm
y diez muestras por visita. Sus posiciones, transformación y parámetros de
robot/sensor son sintéticos; no representan una calibración ni valores de
operación del laboratorio. `m1_lab.json` aún no existe.

## Esquema

Se exigen `schema_version=1`, `name`, `mode="simulation"`, `units` y `geometry`.
Las unidades son `mm`, `deg` y `T`. Se rechazan campos desconocidos o faltantes,
claves JSON duplicadas, números no finitos y tipos incompatibles.

| Sección | Parámetros |
|---|---|
| `geometry` | Radio y paso de la malla; radio y longitud del BORE de referencia. `radio/paso` debe ser entero y la esfera debe caber en el BORE. |
| `acquisition` | Estabilización, muestras por visita, plazos de movimiento/lectura, tolerancias de posición/ángulo, intervalo de llegada estable y máximo de reintentos del sensor. |
| `simulation` | Semilla, pose inicial, transformación, herramienta, modelo escalar de campo, movimiento, intervalo/unidad de muestras y fallos inyectables. |

`plan` admite perfiles sin `acquisition` ni `simulation`, o con ambas secciones
en `null`. `simulate` exige ambas completas. Si una sección está presente,
se valida también al generar un plan.

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
de simulación necesarios. Con `geometry.step_mm=10` el piloto tiene 123 puntos,
8 referencias adicionales, 131 visitas y 1.310 muestras sin reintentos.
Con paso de 5 mm hay 925 puntos, 14 referencias, 939 visitas y 9.390 muestras.
La malla de paso 2,5 mm tiene 7.153 puntos. La retícula previa al recorte se
limita a 2.000.000 de candidatos; se rechazan pasos que excedan ese límite.

No completar direcciones, calibraciones, velocidades ni parámetros físicos
desconocidos con valores supuestos. La integración real requiere su perfil y
adaptadores propios, aún pendientes.
