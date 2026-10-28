# Mapeo magnético con Dobot M1 Pro

Proyecto para recorrer una región esférica de 60 mm de diámetro y asociar
mediciones de campo a cada posición de una sonda. El alcance y los criterios
de aceptación están en el [plan vigente para M1 Pro](Plan_Codex_M1Pro_DSV_60mm.md).
El [plan anterior del M1](Plan_Codex_M1_DSV_60mm.md) se conserva como histórico.

## Estado actual

Están implementadas la planificación geométrica y la adquisición simulada:
robot y sensor sintéticos, confirmación de llegada, referencias centrales,
muestras reproducibles, reintentos acotados, resultados parciales y gráficos.
`plan` y `simulate` funcionan sin hardware ni SDK del fabricante. El modelo de
laboratorio confirmado es **Dobot M1 Pro**. `inspect-robot` permite inspección
de consulta por TCP/IP; `scan --mode hardware` valida requisitos y todavía no
ejecuta adquisición física. No se ha probado el robot del laboratorio.
Los detalles del adaptador y su revisión de referencia están en
[docs/M1PRO_SDK.md](docs/M1PRO_SDK.md).

## Instalación y pruebas

Se requiere Python 3.11 o posterior, con `venv` y `pip`. Desde la raíz del proyecto:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[dev]'
python -m pip check
python -m pytest
```

La instalación editable usa `src/`. Las dependencias de ejecución son NumPy y
Matplotlib; `dev` añade pytest. Los gráficos usan Agg y no necesitan pantalla.

## Generar una trayectoria

```bash
python -m field_mapper plan --config configs/simulation_dsv60.json
```

El perfil usa radio de 30 mm, paso de 5 mm y BORE de referencia de radio 50 mm
y longitud 254 mm. Produce 925 puntos de malla en 13 capas horizontales, de
altura física creciente, guardados en un directorio nuevo `runs/plan-<identificador>/`. Para elegir otro directorio nuevo:

```bash
python -m field_mapper plan --config configs/simulation_dsv60.json --output-dir runs/mi-plan
```

El plan contiene `config_used.json`, `path.csv`, `trajectory.png`, `metadata.json`
y `summary.json`. Incluye la malla y su recorrido; las referencias centrales
se incorporan al ejecutar `simulate`. No incluye rutas físicas de entrada/salida.

La retícula global se define en `(u, v, h)`, incluye centro y frontera y se
recorre de abajo hacia arriba: del punto `(0, 0, -30)` al `(0, 0, +30)` mm.
Se completan todos los puntos de cada altura antes de subir; las filas se
ordenan en `v` y el sentido de `u` alterna. La figura muestra las 13 capas
horizontales y una flecha `+h`, junto con la ROI, el BORE y los extremos.

`h` representa la vertical física. En el montaje de referencia,
`(x_imán, y_imán, z_imán) = (h, u, v)`; el eje del BORE sigue siendo `z_imán`.
El perfil sintético convierte `+h` a `+Z_base` mediante `T_base_magnet`.
Las matrices y el vector `scan.up_base` se validan antes de exportar o mover.

`point_id` depende de los índices `(iu, iv, ih)` y debe interpretarse junto con
el paso y la base de barrido del perfil. `path.csv` conserva esos índices,
`u_scan_mm`, `v_scan_mm`, `h_scan_mm` y las tres coordenadas del imán.
Su `visit_index` comienza en cero y enumera solo la malla.

## Ejecutar la adquisición simulada

```bash
python -m field_mapper simulate --config configs/simulation_dsv60.json
```

Cada visita transforma el objetivo a brida o TCP, ordena el movimiento, confirma
estado detenido y pose dentro de tolerancias durante un intervalo estable,
espera la estabilización configurada y solicita muestras nuevas. Un comando
aceptado por sí solo no autoriza la lectura. El sensor se evalúa en la posición
del punto sensible reconstruida desde la pose reportada del robot simulado.

Las referencias al centro son opcionales y se realizan únicamente **antes y
después** de la malla, mediante `scan.pre_reference` y `scan.post_reference`.
No hay retornos al centro entre capas. Las fases `pre_reference`, `grid_scan`
y `post_reference` y sus transiciones se registran por separado; la monotonía
de altura se verifica dentro de la malla. Cada fase empieza después de confirmar
la llegada a su primer punto: la entrada descendente al extremo inferior y el
retorno final al centro tienen `current_phase="transition"`. Los metadatos
separan `runtime_phases` de `runtime_transitions`. El registro de una transición
simulada no valida una ruta de entrada o retirada para el equipo real.

Con diez muestras por visita y sin fallos ni reintentos:

| Paso | Puntos únicos | Referencias adicionales | Visitas | Muestras |
|---|---:|---:|---:|---:|
| 5 mm, ambas referencias | 925 | 2 | 927 | 9.270 |
| 5 mm, sin referencias | 925 | 0 | 925 | 9.250 |
| 10 mm, piloto con ambas referencias | 123 | 2 | 125 | 1.250 |
| 10 mm, piloto sin referencias | 123 | 0 | 123 | 1.230 |

Para desactivar referencias, establecer ambos indicadores en `false` en una
copia local del perfil. Cada indicador puede activarse de forma independiente.

Para el piloto, copiar `configs/simulation_dsv60.json` a
`configs/pilot.local.json`, cambiar `geometry.step_mm` a `10` y ejecutar:

```bash
python -m field_mapper simulate --config configs/pilot.local.json
```

Los tiempos de movimiento, estabilización y lectura avanzan un reloj virtual
compartido; no esperan minutos de tiempo real. `*_time_s` y
`virtual_duration_s` expresan ese tiempo virtual. `*_utc` registra el reloj
de pared en UTC; `wall_duration_s` mide la duración real de la ejecución.

El modelo produce el escalar `B_n`, con valor base, gradiente lineal, términos
cuadráticos por eje, deriva temporal y ruido normal con semilla configurada.
No reconstruye un campo vectorial ni su magnitud. La misma configuración y
semilla reproducen los datos y tiempos virtuales; los identificadores de
ejecución y tiempos UTC cambian. Se conserva la unidad original `T` o `G` y se
normaliza con `1 G = 1e-4 T`. La media se calcula con el lote válido completo;
la desviación estándar es muestral (`ddof=1`). Con una sola muestra la desviación
queda ausente y el gráfico no inventa una barra de error.

La transformación, el desplazamiento de herramienta, la orientación, la posición
inicial y todos los parámetros del perfil son sintéticos. La etiqueta
`calibration.synthetic=true` de la adquisición identifica esos parámetros;
no corresponde a una calibración del laboratorio.

## Resultados y progreso

`simulate` crea `runs/simulate-<identificador>/`. También acepta `--output-dir`
con un directorio que todavía no exista. Nunca reutiliza resultados anteriores.

| Archivo | Contenido |
|---|---|
| `config_used.json` | Configuración efectiva, incluidos cambios de la CLI. |
| `path.csv` | Índices `(iu,iv,ih)` y coordenadas de barrido/imán de los puntos únicos, sin referencias adicionales. |
| `measurements.csv` | Un registro por intento de visita: fase, tipo, objetivo en barrido/imán, altura de capa, orden a la base, pose reportada, punto sensible reconstruido, tiempos, estado y estadísticas. |
| `samples.csv` | Muestras individuales, valor/unidad originales, valor normalizado, secuencia, tiempos, evidencia de frescura y rechazos. |
| `events.jsonl` | Fases y transiciones, conexiones, movimientos, llegada, lecturas rechazadas, reintentos y cierre. |
| `progress.json` | Último punto de control, actualizado tras cada visita completada y al finalizar. |
| `metadata.json` | Identidades simuladas, versiones y revisión del código, unidades, marcos, política de referencias, fases, huellas SHA-256, parámetros sintéticos y estado. |
| `summary.json` | Estado final, contadores, errores, duraciones, estadísticas de malla y cambio entre referencias inicial/final. |
| `trajectory.png` | ROI, BORE de referencia, capas horizontales, flecha vertical y recorrido ascendente de malla. |
| `field_maps.png` | Cortes centrales discretos de `B_n`, sin interpolación. |
| `field_profiles.png` | Perfiles centrales con desviación muestral cuando está disponible. |
| `reference_drift.png` | Referencias centrales frente al tiempo virtual. |

`visit_id` y `visit_index` de la adquisición incluyen las referencias y comienzan
en cero; `grid_visit_index` enlaza con el recorrido de `path.csv`. `attempt`
comienza en uno y distingue los lotes reintentados dentro de una visita.
Una referencia repite el `point_id` central, pero tiene su propio `visit_id`.
El campo `phase` de una medición identifica su fase prevista. Si falla la
transición de entrada, se conserva la fila fallida de la visita y su fase de
adquisición no aparece iniciada en `runtime_phases`.

`valid_samples` cuenta muestras aceptadas, incluidas las de un lote que después
falle. `used_samples` cuenta solo muestras de lotes completos que aportaron una
media válida. Al reintentar una lectura se solicita un lote nuevo y no se mezclan
muestras del intento fallido. `measurement_attempts` cuenta las escrituras
confirmadas de filas de `measurements.csv`, incluidas las fallidas;
`samples_attempted` cuenta solicitudes de muestra.

Los CSV y eventos se vacían tras cada fila y los JSON de estado se reemplazan
atómicamente. Si una escritura de medición falla, `uncertain_measurement_writes`
advierte que la fila pudo haberse escrito antes del error de E/S. Ese append
no se repite a ciegas ni se suma como confirmado. Conviene revisar CSV, eventos
y resumen juntos cuando exista esa incertidumbre.

El estado final es `complete`, `failed` o `cancelled`. Un `metadata.status`
que siga en `running` puede corresponder a un proceso activo o a una interrupción
antes del cierre; permite detectar un resultado sin finalización confirmada.
Los gráficos usan únicamente mediciones válidas y pueden contener paneles sin
datos en una ejecución parcial. No hay reanudación automática.

## Demostrar fallos y reintentos

Fallo persistente de movimiento en la visita de índice 3, contando referencias:

```bash
python -m field_mapper simulate --config configs/simulation_dsv60.json --fail-motion-at-visit 3
```

Fallo persistente del sensor desde la petición de lectura número 21:

```bash
python -m field_mapper simulate --config configs/simulation_dsv60.json --fail-sensor-at-read 21
```

Los fallos predeterminados son `timeout`. `--motion-fault` admite `timeout`,
`out_of_tolerance`, `connection_loss` y `accepted_only`; este último conserva
el estado en movimiento aunque la pose llegue al objetivo. `--sensor-fault`
admite `timeout`, `stale`, `invalid`, `unit` y `connection_loss`.
Las opciones requieren su índice de fallo, definido en la CLI o en el perfil.

Para comprobar la recuperación de un timeout transitorio con el reintento del perfil:

```bash
python -m field_mapper simulate --config configs/simulation_dsv60.json --fail-sensor-at-read 21 --sensor-fault-once
```

Ante fallo de movimiento no se mide ni se envía otro destino. Ante error de
lectura recuperable se reintenta el lote en la misma posición hasta el límite
configurado. La pérdida de conexión finaliza la secuencia. Al fallar o cancelar
se intenta detener el robot simulado y se cierran las conexiones, conservando
lo registrado; desconectar no ejecuta un retorno ni `home`.

Los códigos de salida de la CLI son `0` al completar, `1` ante un fallo durante
la adquisición, `2` ante argumentos, configuración o salida inicial inválidos,
y `130` al cancelar la adquisición mediante interrupción del operador.

## Coordenadas y límites de la demo

El origen es el centro del BORE. El marco del imán conserva `x` vertical en
el montaje de referencia y `z` longitudinal al BORE; `y` completa la mano derecha.
El marco de barrido `(u, v, h)` permite declarar la vertical física cuando el
montaje está girado. La figura 3D se dibuja en este marco, con `h` vertical.
No se presupone que B₀ siga el BORE.

`RigidTransform` valida rotación y traslación rígidas. `ToolTransform` convierte
entre posición del punto sensible en el imán y pose de brida o TCP; en modo
`tcp` no vuelve a restar el desplazamiento. Las pruebas verifican esta matemática
y la adquisición simulada, incluida la conservación de resultados ante fallos.
No verifican alcance, colisiones, vibración ni perturbación del campo por el robot.

La calibración física, el protocolo del sensor real, las tolerancias y
velocidades de operación y las rutas de entrada/retirada siguen pendientes.
La versión exacta de DobotStudio y el firmware del laboratorio también son
desconocidos; no se deducen del modelo confirmado.

## Inspeccionar el M1 Pro y validar el perfil de laboratorio

```bash
python -m field_mapper inspect-robot --config configs/m1pro_lab.json
python -m field_mapper scan --config configs/m1pro_lab.json --mode hardware
```

El perfil de laboratorio conserva `host=null` y firmware desconocido.
`inspect-robot` rechaza ese perfil antes de conectar: primero hay que introducir
la IP real en una copia local. La inspección consulta estado, pose e identidad
disponible; no habilita motores, mueve, limpia alarmas, reinicia ni ejecuta home.
Antes de conectarse, el operador debe verificar el modo TCP/IP secondary
development indicado por la documentación del controlador y contrastarlo con
la versión exacta del programa instalado. La aplicación no cambia ese modo.

`scan --mode hardware` es una puerta de validación explícita: el perfil de
ejemplo incompleto se rechaza y el comando todavía no ejecuta adquisición
real, incluso si se completan los datos del robot. Faltan la integración del
sensor y las rutas físicas verificadas. No usar el perfil sintético para
completar esos datos. Consultar [la documentación del adaptador](docs/M1PRO_SDK.md)
para la revisión del SDK, campos requeridos y capacidades verificadas sin robot.

## Estructura

```text
src/field_mapper/    Geometría, trayectoria, adaptadores, adquisición y exportación
configs/            Perfiles sintéticos y plantilla incompleta de laboratorio
docs/               Referencia y límites del adaptador TCP/IP M1 Pro
tests/              Pruebas de geometría, simulación, fallos, persistencia y CLI
pyproject.toml      Metadatos, dependencias y configuración de pytest
```

`.venv/`, `runs/` y `configs/*.local.json` se excluyen de Git.
