# Bitácora del RBOT físico

Actualización: 2026-11-02. Este documento conserva las decisiones, la evidencia
y el siguiente ensayo al retomar la sesión. La solución sigue teniendo
**un launch y un nodo operativo**; las pruebas no forman parte del controlador.

## Sesión 2026-11-02 (resumen; detalle más abajo)

1. Lectura sin movimiento: conexión correcta (scan 10 Hz, odom 20 Hz,
   dock_status 1 Hz, TF del láser esperado, relojes NTP, edad de scan ≈0,1 s).
   **No existe `/hazard_detection` en el RBOT.**
2. **Calibración**: robot acoplado a mano y cargando → centro a **0,2373 m**
   de la pared (σ 0,19 mm). `d_floor=0.245` paraba 7,7 mm antes del contacto.
3. **Corrida autorizada** desde d=0,994 m (lateral +6,7 mm, rumbo +4,2°) con
   `d_floor=0.231`, `v_dock=0.035`, `v_fine=0.04`, `v_max=0.08`: llegó
   alineada (e=−0,9 mm, rumbo +1,3°) a WAIT_DOCK en 22,3 s; quieta quedó en
   0,237 m, lateral −0,2 mm, rumbo −0,8°, pero **`is_docked=false`** (también
   >10 s después). Sin choques ni retrocesos. Un primer intento abortó en
   `check_params` (`seat_min_distance` ≥ `d_floor`) sin mover el robot.
4. **Empuje manual** desde esa pose: para acoplar, Patrick tuvo que **girar**
   el robot. Odometría: +8,3 mm y −5,9° hasta el primer true; el LiDAR solo
   vio 1 mm de avance → ruedas patinando contra el dock. Pitch casi igual
   (+0,2°): se descarta "subir la rampa". Pose acoplada: rumbo **+3,6°**
   (calibración: +2,6°) respecto a las cajas.
5. **Conclusión**: el dock físico está girado ~3° respecto al marcador; la
   navegación y la distancia ya eran correctas.
6. **Cambios** (sin robot, tras terminar la reserva): estado **SWEEP**
   (barrido de rumbo por pulsos con pausas), defaults `d_floor=0.231`,
   `seat_min_distance=0.230`, `v_dock=0.035`; 79 tests aprobados.
   **SWEEP no tiene validación física todavía.**

Evidencia: [pose al retomar](pruebas/2026-11-02-pose-actual-solo-lectura.json),
[calibración acoplado](pruebas/2026-11-02-calibracion-dock-manual.json),
[corrida y empuje manual](pruebas/corrida-20261102-194837-ubgl/RESUMEN.json).

## Estado al retomar (antes de la sesión del 2026-11-02)

- El resultado final de la última corrida física **no se ha recuperado**.
  Los archivos temporales citados en la conversación anterior ya no están
  disponibles. No se registra esa corrida como éxito ni como fallo confirmado.
- Se conservan el código corregido y las pruebas. La verificación actual dio
  **64 pruebas aprobadas**, sin contexto DDS ni publicadores hacia el robot.
- Nodo y launch instalados coinciden byte a byte con las fuentes revisadas.
- En una consulta nueva de 6 s: 49 scans, 89 mensajes de odometría y cinco
  estados `is_docked=false`. No apareció un nodo `dock_lidar` activo.
- Se observaron 33 comandos ajenos en `/cmd_vel`, 25 distintos de cero; sus
  publicadores anunciados eran `rosbridge_websocket` y el puente Zenoh. La
  odometría final indicó velocidad cero. Esto describe solo esa ventana y
  no demuestra cómo terminó la corrida anterior.
- La consulta fue de solo lectura: no creó un publicador de velocidades.
  Evidencia: [consulta ROS](pruebas/2026-11-02-consulta-solo-lectura.json) y
  [verificación local](pruebas/2026-11-02-verificacion.json).

## Entorno y alcance

Ubuntu 22.04 nativo, ROS 2 Humble, Husarnet y puente Zenoh al RBOT de Kalman
Robotics: iRobot Create 3 con RPLIDAR A1. `/scan` llega aproximadamente a
10–11 Hz, con 720 rayos y frame `rplidar_link`; se manda `Twist` a `/cmd_vel`.
`/dock_status` se observó aproximadamente a 1 Hz. El montaje del láser se
resuelve mediante TF; no se presupone que el primer rayo apunte al frente.

Se conserva el detector geométrico de las dos cajas, el filtro del eje,
`follow_axis` y el control de rumbo que ya funcionaba. El repositorio de
referencia `create3_dock_challenge` permanece sin modificaciones locales.
No se utiliza la acción automática `/dock` ni los infrarrojos para guiarse.

## El problema y la corrección

Antes, llegar a `d_floor=0.245 m` podía provocar `DOCK → BACKOFF`, aunque el
error lateral fuese de pocos milímetros. Un `true` recibido durante el
retroceso podía terminar la corrida sin comprobar la estabilidad del contacto.

Ahora, al alcanzar ese límite alineado, se entra en **WAIT_DOCK**, con
velocidad cero durante hasta 3 s. No se inicia un retroceso por agotar esa
espera. Los pulsos experimentales están desactivados por defecto.

El primer `is_docked=true` válido detiene la maniobra, también durante un
retroceso, y activa **CONFIRM**. Para declarar **DONE** se requieren al menos
tres mensajes nuevos `true` que abarquen 2 s; esperar con una única muestra
no basta. Un `false` reinicia la confirmación. En DONE se siguen publicando
ceros: perder el contacto o la frescura del estado lleva a **STOPPED**, sin
reanudar movimiento. Un contacto tardío en STOPPED solo se informa.

Se detectó además una discrepancia en la distancia final: durante una corrida
el filtro alcanzó aproximadamente 0.244 m, pero una observación posterior
del detector, con el robot casi inmóvil, dio aproximadamente 0.260 m.
La distancia filtrada también cambió varios milímetros después de parar,
mientras la odometría apenas cambió. Esto apunta a un sesgo o retardo de la
estimación normal al muro; aún no prueba una causa única.

La corrección usa, en el tramo final, la **mediana de hasta tres planos
recientes aceptados del LiDAR**, transformados a `odom` y evaluados con la
posición actual del robot. Así se compensa el movimiento desde cada scan.
El lateral y el rumbo mantienen la estimación y el control anteriores.
Los logs comparan `d_filtro`, `d_lidar` y su diferencia. La corrección está
cubierta por pruebas, pero su eficacia física sigue pendiente de confirmar.

## Límites y paradas implementados

| Condición | Comportamiento actual |
|---|---|
| Cualquier estado, incluido BACKOFF/ESCAPE | Tope absoluto de 0.20 m/s lineal y 0.50 rad/s angular; también se respetan límites configurados inferiores. |
| Comando no finito | Publica ceros. |
| `dry_run=true` | Todas las velocidades de este nodo son cero. |
| LiDAR o TF con edad mayor de 0.75 s | STOPPED tras haber arrancado; hasta 15 s de espera inicial. |
| Sin detección aceptada durante más de 3 s | STOPPED si ya existía una estimación. En el tramo final se exige además un plano reciente, con edad máxima de 0.75 s. |
| Sin dock_status fresco durante más de 2.5 s | STOPPED. Se comprueban recepción y sello de tiempo. |
| Reloj ROS detenido | El temporizador de seguridad usa reloj monotónico. |
| DOCK durante 12 s sin finalizar | WAIT_DOCK parado. |
| Desalineación o BUMP durante aproximación | Recuperaciones limitadas por contador y por tiempo. |
| BUMP durante WAIT_DOCK/SEAT | STOPPED, sin retroceso automático. |
| Recuperación | Máximo 12 s por maniobra. |
| Corrida completa | 180 s por defecto; 90 s en el próximo ensayo aislado. |
| STOPPED | Mantiene ceros hasta relanzar; no se rearma al volver los datos. |
| `dock_push_s` | Se conserva por compatibilidad, pero se ignora. |

El presupuesto global por defecto es de dos recuperaciones adicionales;
también existe un límite de tres eventos BUMP. No son presupuestos que se
sumen. Un BUMP persistente cuenta como un evento. No se ignoran nuevos
choques al agotar el presupuesto: se detiene.

En lecturas anteriores del firmware se observaron `min_speed=0.03 m/s`,
`safety_override=full` y `REFLEX_BUMP=false`; no se modificaron esos parámetros.
No se comprobó si un comando inferior al mínimo es ignorado o transformado.
Por ello `max_seat_steps=0` se mantiene en nodo y launch: los pulsos a
0.01 m/s no tienen validación física. Tampoco se observó un publicador de
`/hazard_detection` en el diagnóstico anterior; suscribir ese tópico no
demuestra que la protección por BUMP esté disponible en esta conexión.

## Configuración de referencia y ensayo aislado

| Parámetro | Defecto actual | Próximo ensayo frontal |
|---|---:|---:|
| `use_sim_time` | false | false |
| `range_sigma` | 0.01 | 0.01 |
| `v_max` | 0.14 | 0.08 |
| `v_fine` | 0.05 | 0.04 |
| `v_dock` | 0.035 (antes 0.025) | 0.035 |
| `v_backoff` | 0.12 | 0.06, sin recuperaciones habilitadas |
| `k_w`, `k_i` | 1.0, 0.0 | 1.0, 0.0 |
| `w_max`, `w_search` | 0.4, 0.3 | 0.4, 0.3 |
| `d_fine` | 0.75 | 0.80 |
| `d_floor` | 0.231 (antes 0.245) | 0.231 |
| `dock_wait_s`, `dock_confirm_s` | 3.0, 2.0 | 3.0, 2.0 |
| `max_dock_retries` | 2 | 0 |
| `max_seat_steps` | 0 | 0 |
| `dock_sweep` (nuevo) | true | true |
| `seat_min_distance` | 0.230 (antes 0.235) | 0.230 |
| `run_timeout_s` | 180 | 90 |

Los valores 0.14/0.05 m/s permanecen como referencia del usuario. El ensayo
a 0.08/0.03 m/s busca aislar la llegada y confirmar contacto antes de volver
a comparar tiempos. No se cambia la distancia mínima para compensar a ciegas
un fallo de acoplamiento.

## Historial conservado de la conversación

Los datos de esta sección proceden del usuario y del resumen de la sesión
anterior. Los logs temporales originales no están disponibles ahora; las
cifras no constituyen una nueva medición ni una tasa de éxito reproducida.

| Prueba anterior del usuario, en WSL | v_max / v_fine | Resultado comunicado |
|---|---|---|
| 46.60 s | 0.08 / 0.03 | Docking exitoso. |
| 32.20 s | 0.14 / 0.05 | Detectó docking, con BACKOFF previo. |
| 25.90 s | 0.14 / 0.05 | Docking exitoso, confirmado posteriormente. |
| 29.20 s | 0.17 / 0.06 | Detectó docking; posteriormente `is_docked=false`. |

Las poses iniciales no fueron idénticas. El log antiguo de 46.60 s corresponde
a **una sola corrida**; no valida las modificaciones actuales.

| Sesión física posterior | Observación conservada | Interpretación |
|---|---|---|
| Colocación manual con carga | El usuario confirmó carga; dry-run sobre robot ya acoplado confirmó true estable. Distancia mediana filtrada ≈0.24368 m, lateral ≈+2.58 mm, rumbo ≈−3.60°. | Valida recepción del estado y confirmación estando ya colocado; no es un éxito de aproximación autónoma. |
| Primer ensayo frontal autorizado | Inicio d≈0.923 m, lateral −7.6 mm, rumbo −1.52°. WAIT_DOCK alrededor de 22.7 s y STOPPED alrededor de 25.8 s desde el arranque del observador. No hubo true ni retrocesos; v máxima observada ≈0.080 m/s. | Fallo de acoplamiento observado; motivó revisar la distancia filtrada. |
| Observación posterior inmóvil | Detector ≈0.260 m frente a ≈0.244 m del filtro al detenerse. Proyectar la inclinación completa explicó aproximadamente 3 mm en una muestra. | Indicio de sesgo de distancia mayor que el efecto de inclinación evaluado; no se reescribió la geometría. |
| Segundo ensayo, tras corregir distancia | Preflight d≈1.087 m, lateral +17.1 mm, rumbo +2.46°. Último tramo conservado: aproximación de d≈0.577 a 0.469 m a 0.03 m/s. | **Resultado final desconocido tras el corte.** No se extrapola desde ese tramo. |

La señal de batería se observó como `UNKNOWN` incluso con carga confirmada
por el usuario; no se usa ese valor para negar ni confirmar contacto.
Las métricas antiguas de 20/20 en Gazebo pertenecen a otra versión y no
validan la actualización del RBOT físico.

## Comprobaciones sin movimiento

Ejecutadas al retomar: 64 pruebas aprobadas en 0.84 s. Se ejecutan el
constructor y los callbacks de producción con reloj, TF y publicadores
reemplazados por objetos en memoria; no se inicializa DDS.

Cubren confirmación temporal, true transitorio/tardío/duplicado, pérdida de
entradas y del reloj, límites en todos los estados, reintentos, espera final,
opción experimental de pulsos y distancia reciente frente a sesgos del filtro.
También comprueban que nodo y launch tengan los mismos valores por defecto y
que el launch contenga un único nodo.

```bash
source /opt/ros/humble/setup.bash
cd ~/sim_ws/src/terraslam_solucion_dock_lidar
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python3 -m pytest -q -p no:cacheprovider test/test_docking_safety.py
```

La compilación e instalación se habían realizado antes del corte. En esta
reanudación se comprobó que las fuentes y los archivos instalados coinciden;
solo se actualizó documentación, sin editar nodo ni launch.

## Sesión 2026-11-02 (23:05) — SWEEP validado en el robot

Reserva nueva. Verificación por lectura: pose de salida d=0,882 m, lateral
+13,9 mm, `head_err` +11,2° (no exactamente frontal); robot quieto, sin
`/cmd_vel`. Dry-run privado correcto (`pruebas/dryrun-sweep-20261102-230655`).

**Una** corrida autorizada (`pruebas/corrida-20261102-230721-ZFJy`), perfil
lento `v_max:=0.08 v_fine:=0.04 max_dock_retries:=0 run_timeout_s:=90.0`,
resto defaults:

| t [s] | Evento |
|---|---|
| 1,00 | SEARCH → APPROACH |
| 14,00 | APPROACH → DOCK |
| 19,85 | DOCK → WAIT_DOCK (d 0,2365 m, e −1,4 mm, head +2,2°, sin true) |
| 22,90 | WAIT_DOCK → SWEEP |
| 23,0 | objetivo +1,5° → rumbo +1,3° |
| 24,4 | objetivo −1,5° → rumbo −1,8° |
| 25,96 | objetivo +3,0°: true 0,21 s tras iniciar el pulso → CONFIRM |
| 28,00 | CONFIRM → DONE (3 lecturas, 2,01 s) |

Hecho: acopló de forma autónoma y `is_docked` seguía true >30 s después.
Hipótesis: el contacto corresponde al rumbo −1,8° del objetivo 2 (estado a
1 Hz con ~0,2 s de latencia); coincide en signo con la calibración manual.
Pendiente: confirmación de carga física por Patrick.

Cambio posterior (sin efecto en el control): `main()` ignora también
`ExternalShutdownException` y protege `destroy_node()`, para que el Ctrl-C del
launch no imprima una traza (`pruebas/dryrun-salida-*`: "finished cleanly").
79 tests aprobados.

Patrick confirmó carga física tras esta corrida.

**Corrida 2** (`pruebas/corrida-20261102-231118-fZuv`), autorizada, **sin
argumentos** (defaults de entrega, `v_max` 0,14). Pose inicial d=0,942 m,
lateral +37,6 mm, `head_err` +6,0°. APPROACH→DOCK 11,55 s; WAIT_DOCK 15,90 s
(d 0,236, e −1,5 mm, head +1,3°); SWEEP 18,95 s; objetivo −1,5° → −1,1°;
al iniciar el pulso hacia +3° llegó un **true fugaz** (CONFIRM 20,76 →
WAIT_DOCK 21,80); el barrido reanudado siguió hacia +3° (3 intentos hasta
+2,5°) y luego hacia −3°: true → **DONE a 30,8 s**, `is_docked` true después.

Hecho: con defaults acopla. Hipótesis: la zona de contacto está en
`head_err` negativo (−1…−3°); se perdieron ~7 s alejándose de ella tras el
true fugaz.

Cambios después de la corrida 2 (sin robot, 80 tests):
- Tras un true fugaz en SWEEP, al reanudar se vuelve primero al rumbo en que
  el robot estaba quieto cuando llegó ese true (rumbo previo al pulso si
  llegó durante un pulso), máximo 2 veces; luego sigue el barrido. Genérico:
  no fija ningún ángulo. **Sin validar en el robot.**
- `main()` trata como cierre el `RCLError` de publicación con el contexto ya
  invalidado por el Ctrl-C (traza vista al final de la corrida 2).

**Corrida 3** (`pruebas/corrida-20261102-231607-RPdW`), autorizada, defaults,
**pose lateral**: robot paralelo a la pared (rumbo +93° respecto a la
normal), d=0,921 m, lateral −377 mm, detección 10/10. SEARCH→STAGE 1,00 s;
APPROACH 2,16 s; DOCK 14,45 s; WAIT_DOCK 19,30 s (d 0,232, e +0,3 mm, head
+0,6°); SWEEP 22,35 s; +1,5° → +1,0°; −1,5° → −0,7°; true durante el
siguiente pulso → **DONE a 28,55 s**. Salida limpia ("finished cleanly").
Primera corrida física con el retorno al contacto incluido (no se activó: no
hubo true fugaz).

Hipótesis acumulada: llegadas con `head_err` +0,6…+2,2° y contactos en
−0,7…−3°; el barrido los cubre con sus dos primeros objetivos.

Siguiente (cada una con su autorización): pose lateral del otro lado y
rumbos mayores (de espaldas al dock).

Documentación: README (sección "Resultados en el RBOT real") y
SOLUCION_EXPLICADA (sección 9b y C12) con figuras en `docs/img/`, generadas
desde estos logs por `~/sim_ws/dev_tools/rbot/plots.py`.

## Próxima reserva: validar SWEEP (ejecutado el 2026-11-02 23:07, ver arriba)

1. Conectar la reserva y verificar sin movimiento con
   `~/sim_ws/dev_tools/rbot/pose_check.py` (ver su README).
2. Misma pose frontal: centro a ~1 m de la pared, centrado y mirando al dock.
   Sirve aunque sea aproximada: se mide con el LiDAR.
3. Dry-run con salida privada (no mueve): confirmar arranque y `barrido=+-6.0 deg`.
4. Con autorización de **una** corrida: `~/sim_ws/dev_tools/rbot/run_trial.sh`
   con perfil lento (`v_max:=0.08 v_fine:=0.04 max_dock_retries:=0
   run_timeout_s:=90.0`); el resto, defaults (`d_floor` 0,231, `v_dock` 0,035,
   SWEEP activo).
5. Esperado: llegada a WAIT_DOCK ~0,237 m; sin true en 3 s → SWEEP; contacto
   en el objetivo que lleve el rumbo a ≈+3° respecto a las cajas
   (`head_err` ≈ −3° en el log del nodo, por convención de signos). Registrar
   el objetivo y el intento del primer true, y la confirmación (DONE).
6. Si confirma: Patrick verifica carga física; luego repetir con defaults
   completos (`v_max` 0,14) y después poses no frontales.
7. Si SWEEP se agota sin true: conservar el log, medir la pose final y repetir
   el empuje manual con `observer_push.py` antes de cambiar nada. En el
   empuje del 2026-11-02 la odometría avanzó 8 mm con el LiDAR casi inmóvil
   (ruedas contra el dock): si el giro solo no basta, la siguiente prueba
   candidata es más presión, bajando `d_floor` en pasos de 2 mm (nunca por
   debajo de ~0.227: las cajas se tocan a ~0.225 m y `seat_min_distance` debe
   quedar por debajo de `d_floor`), sin pulsos nuevos ni código nuevo.
8. El dry-run del 2026-11-02 con SWEEP (`pruebas/dryrun-sweep-*`) no llegó a
   arrancar: la reserva ya había terminado (falta de `cyclonedds.xml`).

## Siguiente pose y criterio de aceptación (plan anterior, ya ejecutado)

Primero repetir una pose **frontal reproducible**, sin introducir todavía
una salida aleatoria:

1. Controlador anterior detenido y mando de la aplicación en reposo.
2. Centro del robot a **1.0 m aproximadamente de la pared** que sostiene las
   cajas, con margen de 0.9–1.1 m. La referencia es la pared, no el borde de
   la base de carga.
3. Centro alineado con el punto medio del hueco entre las cajas; frente del
   robot mirando al dock, aproximadamente perpendicular a la pared.
4. Objetivo inicial medido: lateral ≤2 cm y rumbo ≤5°. La pose se comprueba
   con LiDAR/TF antes de autorizar una nueva aproximación.

Se realizará **una sola corrida por autorización**. El perfil propuesto es
el de la tabla anterior, sin reintentos ni pulsos. Se exige true estable
según CONFIRM y una observación posterior de al menos 5 s; el operador
comprueba además si quedó cargando. Un true fugaz o estar encima de la rampa
no se registra como éxito estable. Las poses con desplazamiento lateral y
rumbo distintos quedan para después de validar esta llegada frontal.

## Registro de las siguientes corridas

Guardar los resultados **dentro del repositorio**, no únicamente en `/tmp`.
Cada corrida debe incluir versión/hashes, parámetros efectivos, pose inicial
medida, log de transiciones, `d_filtro/d_lidar`, estado final y confirmación
física del operador. Distinguir tiempo hasta el primer true, hasta DONE y
duración posterior de la observación.

Para una ejecución manual ya autorizada, este comando conserva la consola
y los logs ROS en un directorio nuevo y mantiene un launch/un nodo:

```bash
source /opt/ros/humble/setup.bash
source ~/sim_ws/install/setup.bash
cd ~/sim_ws/src/terraslam_solucion_dock_lidar
mkdir -p docs/pruebas
DOCK_TRIAL_DIR="$(mktemp -d "$PWD/docs/pruebas/corrida-XXXXXXXX")"
export ROS_LOG_DIR="$DOCK_TRIAL_DIR/ros"
set -o pipefail
ros2 launch terraslam_solucion_dock_lidar solucion.launch.py \
  use_sim_time:=false range_sigma:=0.01 \
  v_max:=0.08 v_fine:=0.03 v_dock:=0.025 v_backoff:=0.06 \
  k_w:=1.0 k_i:=0.0 w_max:=0.4 w_search:=0.3 d_fine:=0.80 \
  dock_push_s:=0.0 max_dock_retries:=0 max_seat_steps:=0 \
  run_timeout_s:=90.0 log_level:=debug \
  2>&1 | tee "$DOCK_TRIAL_DIR/controller.log"
```

El comando anterior **mueve al robot**; está documentado para la próxima
corrida autorizada, no se ejecutó al escribir esta bitácora. Para evaluar
sin movimiento, añadir `dry_run:=true`; para aislar también la salida del
robot, añadir `cmd_vel_topic:=/terraslam_validation/cmd_vel`.
