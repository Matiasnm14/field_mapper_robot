# Pruebas

Con el entorno activado, ejecutar desde la raíz del proyecto:

```bash
python -m pytest
```

La suite cubre:

- Geometría: conteos de 123, 925 y 7.153 puntos, unicidad, simetría y frontera.
- Transformaciones: ida/vuelta, brida/TCP y reconstrucción del punto sensible
  con rotaciones y desplazamientos sintéticos.
- Configuración: campos estrictos, números finitos, enteros y booleanos,
  unidades, vectores, matrices rígidas, semillas, fallos, JSON ambiguo y
  compatibilidad con perfiles solo de planificación.
- Trayectoria y referencias: recorrido determinista, identificadores estables,
  referencia inicial y otra tras cada capa; 939 visitas con paso de 5 mm y
  131 visitas con paso de 10 mm.
- Adaptadores: reloj virtual, aceptación distinta de llegada, realimentación,
  tolerancias, estabilidad, unidades G/T, modelo escalar reproducible y
  secuencias/tiempos de muestras nuevas.
- Adquisición: llegada antes de medir, comprobación de pose, frescura, lotes y
  reintentos acotados, media/desviación muestral (`ddof=1`), desviación ausente
  con una muestra y distinción entre muestras válidas y utilizadas.
- Fallos: timeout, pose incompatible, aceptación sin finalización, pérdida de
  conexión, muestra antigua, valor inválido y unidad sin conversión validada;
  detención de la secuencia, cancelación y desconexión sin retorno automático.
- Persistencia: CSV y eventos incrementales, JSON de estado, resultados
  parciales, fallos de E/S, conteo de appends confirmados e identificación
  de escrituras inciertas sin repetir la fila a ciegas.
- CLI y exportación: `plan` y `simulate` sin pantalla, configuración efectiva,
  metadatos, huellas, figuras y rechazo de directorios existentes; códigos de
  salida de éxito, fallo, configuración inválida y cancelación.

Para trabajar por área:

```bash
python -m pytest tests/test_simulated_adapters.py tests/test_simulation_config.py
python -m pytest tests/test_acquisition.py tests/test_runner_storage.py
python -m pytest tests/test_scan_storage.py tests/test_simulate_cli.py
```

Las pruebas usan software y parámetros sintéticos. No certifican alcance,
colisiones, seguridad física, exactitud de un sensor real ni comportamiento del
SDK del M1. Los adaptadores reales, `inspect-robot` y `scan` siguen pendientes.
