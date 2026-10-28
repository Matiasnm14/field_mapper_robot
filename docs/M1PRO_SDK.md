# Adaptador TCP/IP del Dobot M1 Pro

El adaptador usa el protocolo oficial de cuatro ejes de MG400/M1Pro y se prueba
con sockets en memoria. No se conectó un robot ni se ejecutó el ejemplo oficial.
No usa la DLL Win32 del M1 original ni APIs V4 de otros modelos.

## Fuentes fijadas y licencia

- [SDK Python oficial, revisión b5f95647cfc44a60f92babf5a464d38dd1e4f72c](https://github.com/Dobot-Arm/TCP-IP-4Axis-Python/tree/b5f95647cfc44a60f92babf5a464d38dd1e4f72c).
- [dobot_api.py en esa revisión](https://github.com/Dobot-Arm/TCP-IP-4Axis-Python/blob/b5f95647cfc44a60f92babf5a464d38dd1e4f72c/dobot_api.py): clases Dashboard/Move, consultas, `MovL`, `Sync` y `EmergencyStop`.
- [README fijado](https://github.com/Dobot-Arm/TCP-IP-4Axis-Python/blob/b5f95647cfc44a60f92babf5a464d38dd1e4f72c/README.md): familia MG400/M1Pro, controlador V1.5.6.0 o posterior y configuración del modo TCP/IP secondary development en DobotStudio Pro.
- [Manual oficial TCP/IP cuatro ejes, edición 2024-04-19](https://github.com/Dobot-Arm/TCP-IP-Protocol-4AXis/blob/be74b056be664d1a1a0c95c89e4ecdf950ff3ecf/TCP_IP%20Remote%20Control%20Interface%20Guide%20%284axis%29_20240419_en.pdf), revisión del repositorio de protocolo `be74b056be664d1a1a0c95c89e4ecdf950ff3ecf`.

El SDK tiene licencia MIT, © 2022 Dobot. Se conserva su texto en
[LICENSE](../vendor/dobot_tcpip_4axis/LICENSE) y el registro de archivos revisados,
huellas y procedencia en [provenance.json](../vendor/dobot_tcpip_4axis/provenance.json).
El adaptador es una implementación propia del subconjunto documentado; no
importa ni distribuye el código ejecutable del SDK. El PDF se revisó sin
redistribuirlo ni extenderle la licencia MIT del repositorio Python.

Las páginas citadas abajo son las numeradas en el documento: descripción de
puertos y ASCII, pp. 4–5; cola y sincronización, p. 6; parada de emergencia,
p. 28; modos, p. 29; pose, pp. 30–31; alarmas, p. 31; parámetros de movimiento,
p. 49; `MovL`, p. 52; `Sync`, p. 58; feedback, pp. 62–68.

El historial del PDF (p. 3) relaciona la edición con V1.6.0.0; eso no establece
el firmware mínimo de cada parámetro. El mínimo general declarado por el SDK
es V1.5.6.0. El perfil exige una versión declarada dentro de ese rango y
confirmación con evidencia de las capacidades del controlador instalado.
La referencia del README a V1.6.0.0 en resolución de problemas no constituye
una instrucción para actualizar el laboratorio. No se actualiza firmware ni
se cambia el modo del controlador automáticamente.

## Configuración pendiente

[configs/m1pro_lab.json](../configs/m1pro_lab.json) es un perfil independiente
de los perfiles geométricos/simulados. Tiene `host=null`, `firmware_version=null`,
`compatible_confirmed=false`, `motion_enabled=false` y `calibration=null`.
No contiene una IP, calibración, velocidad física ni firmware supuestos.
Los puertos documentados son `dashboard_port=29999`, `move_port=30003` y
`feedback_port=30004`; son configurables y deben contrastarse en el laboratorio.
Los timeouts locales de comunicación del perfil no representan una puesta a
punto de movimientos.

`load_m1pro_config(path)` carga un `M1ProConfig` estricto: rechaza campos
faltantes/desconocidos, claves duplicadas, números no finitos y tipos inválidos.
Importar los módulos, cargar el perfil o construir `DobotM1Pro(config)` no
abre conexiones. `plan` y `simulate` no usan este adaptador.

`M1ProConfig.preflight_movement()` comprueba las declaraciones necesarias sin
conectar: host, firmware declarado, compatibilidad y evidencia, movimiento
habilitado explícitamente en el perfil, velocidad/aceleración puestas a punto,
calibración y rutas confirmadas, y procedimiento de parada de emergencia
confirmado. No verifica la veracidad física de esas declaraciones.

La calibración completa requiere estos campos:

| Campo | Interpretación |
|---|---|
| `confirmed`, `source` | Confirmación y referencia al procedimiento/registro físico. |
| `T_base_magnet` | Transformación rígida 4×4 desde el imán a la base. |
| `rotation_base_flange` | Rotación propia 3×3 del montaje fijo. |
| `offset_flange_sensor_mm` | Desplazamiento de tres componentes del punto sensible. |
| `pose_frame` | `flange` o `tcp`, coherente con el Tool del controlador. |
| `fixed_angle_deg` | Coordenada R fija durante el barrido. |
| `sensitive_axis_magnet` | Eje sensible unitario expresado en el imán. |
| `user_index`, `tool_index` | Índices declarados: solo User=0/base está implementado; Tool admite 0–9. |
| `coordinate_frames_verified`, `paths_verified` | Confirmación de marcos y rutas/segmentos, incluida entrada y retirada. |

`speed_l_percent` y `acc_l_percent` son porcentajes del protocolo, entre 1 y 100;
no son mm/s ni mm/s². El manual (p. 49) explica que el resultado depende de
los ajustes de reproducción y del porcentaje global. La puesta a punto debe
verificar también esos ajustes, el montaje y los segmentos reales.

## Inspección de solo lectura

`robot.connect()` abre únicamente Dashboard. No envía instrucciones ni abre
movimiento/feedback. `inspect_robot(config)` conecta, consulta `RobotMode()`,
`GetPose()` y `GetErrorID()`, devuelve un diccionario y cierra los canales en
un bloque `finally`. No envía `EnableRobot`, `ClearError`, `Continue`, reset,
home, cambios de modo ni `EmergencyStop`.

Con calibración declarada, la pose se pide con `GetPose(User=0,Tool=<índice>)`;
sin ella se informa la pose bajo los marcos globales activos del controlador,
indicando que no están calibrados/verificados. El eco de una orden solo vincula
la respuesta con la petición: no identifica el modelo ni el firmware.

No se verificó una consulta de identidad/firmware en este subconjunto del SDK.
Por eso `identity.model` e `identity.firmware_version` son `null`, acompañados de
su limitación. `target_model` y `firmware_version_declared` preservan lo declarado
en el perfil, por separado. Una conexión TCP y consultas exitosas no prueban
compatibilidad completa. La versión exacta de DobotStudio observada en el
laboratorio también sigue pendiente.

## Movimiento protegido y confirmación

El código conserva el contrato interno `RobotAdapter`, pero las operaciones
físicas requieren además autorización explícita por llamada:

```python
robot.move_to(position_mm, angle_deg, authorize=True)
```

La llamada exige el preflight completo, Dashboard conectado, ausencia de códigos
de alarma y `RobotMode=5` (habilitado y detenido). Nunca habilita el robot.
Verifica que R coincida con `fixed_angle_deg`, abre el canal de movimiento solo
entonces y usa:

```text
MovL(X,Y,Z,R,User=0,Tool=<índice>,SpeedL=<porcentaje>,AccL=<porcentaje>,CP=0)
```

`CP=0` y la confirmación separada de cada destino evitan encolar un barrido
continuo de puntos desde esta aplicación. No hay `MovJ`, salto ni cambio
automático de configuración del brazo. El adaptador recibe posiciones ya
transformadas; no sustituye al planificador ni a la comprobación de colisiones.

La respuesta de `MovL` confirma aceptación, no llegada. Se mantiene un destino
pendiente y se bloquea cualquier otro hasta confirmarlo. `wait_until_arrived`
exige ese mismo destino, ejecuta `Sync()` en movimiento con plazo absoluto y
solicita después pose, alarmas y modo nuevos por Dashboard. Requiere posición
y R dentro de tolerancias y modo 5 durante el intervalo estable solicitado.
Modo 7 significa movimiento; modo 9, alarma. Pausa, deshabilitado, arrastre,
jogging o modos desconocidos bloquean la operación.

Las consultas ASCII inmediatas proporcionan la pose reportada por el controlador,
no verificación metrológica independiente. No se reutiliza una pose almacenada
para confirmar el destino. Un timeout, error de llegada o estado incompatible
bloquea destinos posteriores, incluso tras reconectar el mismo objeto; no se
reenvía una orden cuyo resultado pudiera ser incierto.

## Transporte y capacidades no disponibles

El transporte envía ASCII mediante `sendall`, acumula fragmentos hasta `;`,
impone límites de mensaje y de tiempo y valida código de retorno, forma y eco.
El lector puede separar mensajes concatenados; un intercambio de petición única
rechaza respuestas adicionales no solicitadas, en lugar de aceptarlas como
estado fresco. `recv` vacío es desconexión. Tras una respuesta incierta, timeout
o error de protocolo, se cierra el canal; no hay reconexión/reintento implícitos.
Las esperas por exclusión del canal también respetan el plazo.

El manual describe feedback binario de 1440 bytes con marcador TestValue y
campos de estado. Su versión/formato en el controlador del laboratorio no se
ha validado. La capacidad `binary_feedback` se declara `false` y no se abre
el puerto 30004. Esta implementación confirma mediante `Sync` y consultas
ASCII; no fabrica ni presenta esas consultas como tramas de feedback binario.

El formato de `GetErrorID` tiene una inconsistencia en p. 31: el literal de
retorno muestra siete grupos y la descripción menciona controlador más cuatro
servos. El lector acepta únicamente cinco o siete grupos de enteros y conserva
su estructura. El documento define un grupo `[]` como ausencia de alarmas;
no documenta `[0]` con ese significado. Para confirmar movimiento se exigen
todos los grupos vacíos; un `[0]`, otro código o un esquema distinto bloquean
la operación. La lista superior vacía `[]` no se interpreta como ausencia de
alarmas. Esto debe contrastarse con respuestas reales durante la puesta a punto.

## Parada y desconexión

No se verificó un comando genérico `Stop()` de movimiento en este SDK.
`StopScript()` detiene proyectos y no se sustituye por parada de una cola
TCP/IP. La operación documentada disponible es `EmergencyStop()`: según el
manual, corta alimentación y produce una alarma, con consecuencias para el
montaje y la sonda. No se ejecuta al conectar, inspeccionar o desconectar.

`stop(authorize=True)` solo puede enviarla si el perfil completo supera
preflight y `emergency_stop_confirmed=true`. El mismo requisito bloquea el
movimiento cuando no se ha confirmado ese procedimiento. Su respuesta de
aceptación no verifica la detención física y no sustituye al paro cableado.
La aplicación no limpia la alarma, no vuelve a habilitar y no continúa.
`disconnect()` solo cierra sockets: no manda movimientos, home ni retirada.

La adquisición real continúa bloqueada hasta integrar sensor real, rutas y
supervisión de operación; los parámetros sintéticos de la demo no se trasladan
al hardware. El programa gráfico no es dependencia de este adaptador, pero
la configuración del controlador debe verificarse con la versión instalada.

## Verificación sin hardware

```bash
python -m pytest tests/test_m1pro_config.py tests/test_m1pro_adapter.py
```

Se verifican inspección sin efectos, cierre de sockets, configuración pendiente,
framing y esquemas, timeouts, aceptación distinta de llegada, estabilidad,
alarmas, pérdida de conexión, orientación fija y guardas de movimiento/parada.
Las pruebas impiden abrir conexiones reales. Se ejecutaron en Linux con Python
3.14.8; no constituyen validación física de Linux, Windows ni del M1 Pro.
