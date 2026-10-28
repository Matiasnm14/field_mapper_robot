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
- Trayectoria: retícula `(u,v,h)`, identificadores estables y recorrido
  determinista por capas horizontales completas, con altura física creciente;
  el montaje sintético convierte `h` a `Z_base`. Conteos de capa, extremos
  inferior/superior y rechazo de transformaciones incompatibles con `up_base`.
- Referencias y fases: las cuatro combinaciones de referencias antes/después,
  sin referencias intercaladas; 927 visitas y 9.270 muestras con paso de 5 mm
  y diez muestras por visita, o 925 visitas y 9.250 muestras sin referencias.
  El piloto de 10 mm usa 125 visitas y 1.250 muestras con ambas referencias.
  Se contrastan las fases `pre_reference`, `grid_scan`, `post_reference`, las
  transiciones y la altura reportada en base. La fase de malla empieza tras
  llegar al extremo inferior y excluye las entradas descendentes de su ventana
  temporal. Un fallo de entrada conserva la visita parcial sin iniciar la fase
  de destino; un fallo o cancelación durante la malla impide la referencia posterior.
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
- CLI y exportación: `plan` y `simulate` sin pantalla; índices `(iu,iv,ih)` y
  coordenadas de barrido/imán, configuración efectiva, metadatos, huellas,
  figuras de capas horizontales con flecha vertical y rechazo de directorios
  existentes; códigos de salida de éxito, fallo, configuración inválida y
  cancelación.
- M1 Pro: validación del perfil de laboratorio y protocolo TCP/IP con datos
  simulados; inspección de consulta sin habilitar, mover, limpiar alarmas ni
  resetear. El firmware no observado se conserva en `null`.
  `scan --mode hardware` valida requisitos y no inicia adquisición física.

Para trabajar por área:

```bash
python -m pytest tests/test_simulated_adapters.py tests/test_simulation_config.py
python -m pytest tests/test_scan_frame.py tests/test_scan_phases.py
python -m pytest tests/test_acquisition.py tests/test_runner_storage.py
python -m pytest tests/test_scan_storage.py tests/test_simulate_cli.py
python -m pytest tests/test_m1pro_config.py tests/test_m1pro_adapter.py
```

Las pruebas usan software y parámetros sintéticos. No certifican alcance,
colisiones, seguridad física, exactitud de un sensor real ni comportamiento del
M1 Pro del laboratorio. La revisión del SDK, los protocolos y sus límites se
registran en [docs/M1PRO_SDK.md](../docs/M1PRO_SDK.md). La inspección requiere
una IP real configurada; la plantilla `host=null` no conecta. El sensor real,
las rutas físicas y la adquisición con hardware siguen pendientes. El plan
vigente es [Plan_Codex_M1Pro_DSV_60mm.md](../Plan_Codex_M1Pro_DSV_60mm.md).
