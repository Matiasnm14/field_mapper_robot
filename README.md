# Mapeo magnético con Dobot M1

Proyecto para recorrer una región esférica de 60 mm de diámetro y asociar
mediciones de campo a cada posición de una sonda. El alcance y los criterios
de aceptación están en [Plan_Codex_M1_DSV_60mm.md](Plan_Codex_M1_DSV_60mm.md).

## Estado actual

Están implementadas la planificación geométrica y la adquisición simulada:
robot y sensor sintéticos, confirmación de llegada, referencias centrales,
muestras reproducibles, reintentos acotados, resultados parciales y gráficos.
`plan` y `simulate` funcionan sin hardware ni SDK del fabricante. Los comandos
`inspect-robot` y `scan` y los adaptadores reales siguen pendientes.

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
y longitud 254 mm. Produce 925 puntos de malla en 13 capas, guardados en un
directorio nuevo `runs/plan-<identificador>/`. Para elegir otro directorio nuevo:

```bash
python -m field_mapper plan --config configs/simulation_dsv60.json --output-dir runs/mi-plan
```

El plan contiene `config_used.json`, `path.csv`, `trajectory.png`, `metadata.json`
y `summary.json`. Incluye la malla y su recorrido; las referencias centrales
se incorporan al ejecutar `simulate`. No incluye rutas físicas de entrada/salida.

La retícula global incluye centro y frontera. El recorrido usa capas en `z`,
filas en `x` y barrido alternado en `y`; entre capas selecciona la entrada más
cercana entre cuatro orientaciones, con desempate determinista.
`point_id` depende de los índices de la retícula y debe interpretarse junto con
el paso del perfil. `visit_index` en `path.csv` comienza en cero y enumera solo
la malla.

## Ejecutar la adquisición simulada

```bash
python -m field_mapper simulate --config configs/simulation_dsv60.json
```

Cada visita transforma el objetivo a brida o TCP, ordena el movimiento, confirma
estado detenido y pose dentro de tolerancias durante un intervalo estable,
espera la estabilización configurada y solicita muestras nuevas. Un comando
aceptado por sí solo no autoriza la lectura. El sensor se evalúa en la posición
del punto sensible reconstruida desde la pose reportada del robot simulado.

Se mide el centro al principio y después de cada capa. Con el perfil de ejemplo
y sin fallos ni reintentos:

| Paso | Puntos únicos | Referencias adicionales | Visitas | Muestras (10 por visita) |
|---|---:|---:|---:|---:|
| 5 mm | 925 | 14 | 939 | 9.390 |
| 10 mm, piloto | 123 | 8 | 131 | 1.310 |

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
| `path.csv` | Puntos únicos de malla en orden de recorrido, sin referencias adicionales. |
| `measurements.csv` | Un registro por intento de visita: referencia o malla, objetivo, orden, pose reportada, punto sensible reconstruido, tiempos, estado y estadísticas. |
| `samples.csv` | Muestras individuales, valor/unidad originales, valor normalizado, secuencia, tiempos, evidencia de frescura y rechazos. |
| `events.jsonl` | Conexiones, movimientos, llegada, lecturas rechazadas, reintentos y cierre. |
| `progress.json` | Último punto de control, actualizado tras cada visita completada y al finalizar. |
| `metadata.json` | Identidades simuladas, versiones, unidades, marcos, huellas SHA-256, parámetros sintéticos y estado de la ejecución. |
| `summary.json` | Estado final, contadores, errores, duraciones, estadísticas de malla y cambio entre referencias inicial/final. |
| `trajectory.png` | ROI, BORE de referencia y recorrido de malla. |
| `field_maps.png` | Cortes centrales discretos de `B_n`, sin interpolación. |
| `field_profiles.png` | Perfiles centrales con desviación muestral cuando está disponible. |
| `reference_drift.png` | Referencias centrales frente al tiempo virtual. |

`visit_id` y `visit_index` de la adquisición incluyen las referencias y comienzan
en cero; `grid_visit_index` enlaza con el recorrido de `path.csv`. `attempt`
comienza en uno y distingue los lotes reintentados dentro de una visita.
Una referencia repite el `point_id` central, pero tiene su propio `visit_id`.

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

El origen es el centro del BORE; `x` es vertical, `z` sigue el BORE e `y` completa
un sistema de mano derecha. La figura 3D usa la permutación cíclica `(y,z,x)`
para mostrar `x` vertical. No se presupone que B₀ siga el BORE.

`RigidTransform` valida rotación y traslación rígidas. `ToolTransform` convierte
entre posición del punto sensible en el imán y pose de brida o TCP; en modo
`tcp` no vuelve a restar el desplazamiento. Las pruebas verifican esta matemática
y la adquisición simulada, incluida la conservación de resultados ante fallos.
No verifican alcance, colisiones, vibración ni perturbación del campo por el robot.

Quedan pendientes la identificación del M1 y su SDK/documentación, el protocolo
del sensor, la puesta a punto de tolerancias y velocidades, la calibración física
y rutas de entrada/salida verificadas antes de implementar operación real.

## Estructura

```text
src/field_mapper/    Geometría, trayectoria, adaptadores, adquisición y exportación
configs/            Perfiles sintéticos y documentación del esquema
tests/              Pruebas de geometría, simulación, fallos, persistencia y CLI
pyproject.toml      Metadatos, dependencias y configuración de pytest
```

`.venv/`, `runs/` y `configs/*.local.json` se excluyen de Git.
