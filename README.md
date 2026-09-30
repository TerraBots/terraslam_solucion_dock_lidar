# terraslam_solucion_dock_lidar — Kalman Dock Challenge (HRFEST 2026)

Docking autónomo del iRobot Create 3 usando **solo el LiDAR**. El robot
reconoce las dos cajas marcadoras por su geometría, **triangula sus bordes**
con precisión sub-muestra, estima el eje del dock y la normal de la pared, y
se acopla en lazo cerrado hasta que `/dock_status` reporta `is_docked: true`.

| Resultado en simulación (20 poses aleatorias del rango del jurado) | |
|---|---|
| Acoplamientos (sin choques) | **20/20** |
| Tiempo medio hasta `is_docked` | **16,7 s** (peor caso 18,9 s) |
| Error lateral medio | **0,10 mm** (peor caso 0,4 mm) |
| Error angular medio | **0,12°** (peor caso 0,24°) |

## Contenido

1. [Equipo](#equipo)
2. [Instalación y build](#instalación-y-build)
3. [Cómo correrlo](#cómo-correrlo)
4. [Parámetros del launch](#parámetros-del-launch)
5. [Cómo funciona](#cómo-funciona)
   - [Cómo sabe el LiDAR que esas son las cajas](#1-percepción-cómo-sabe-el-lidar-que-esas-son-las-cajas)
   - [Triangulación de los bordes](#2-triangulación-sub-muestra-de-los-bordes)
   - [Filtro de Kalman](#3-estimación-filtro-de-kalman-en-odom)
   - [Control](#4-control-en-el-frame-del-dock)
6. [Condiciones consideradas](#condiciones-consideradas)
7. [Validación](#validación)
8. [Limitaciones conocidas](#limitaciones-conocidas)

---

## Equipo

| Nombre Completo | Correo |
|---|---|
| Patrick Fabrizio Echevarria Duran | patrick.echevarria.d@gmail.com |
| Marco Jesus Prado Vasquez | pradovasquezm@gmail.com |
| Camilo Roger Callupe Menejes | camilo.callupe.m@uni.pe |

---

## Instalación y build

Requiere Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11 y el escenario
[`create3_dock_challenge`](https://github.com/Kalman-Robotics/create3_dock_challenge)
en el mismo workspace. La única dependencia extra es **NumPy**, declarada en
`package.xml` e instalada por rosdep.

```bash
cd ~/sim_ws/src
git clone https://github.com/Kalman-Robotics/create3_dock_challenge.git
git clone <este repositorio> terraslam_solucion_dock_lidar

cd ~/sim_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## Cómo correrlo

### Comando único

```bash
# Terminal 1: el escenario del reto (sin tocar)
ros2 run create3_dock_challenge clean_sim.sh
ros2 launch create3_dock_challenge challenge_world.launch.py

# Terminal 2: la solución
ros2 launch terraslam_solucion_dock_lidar solucion.launch.py
```

El nodo arranca solo, se acopla, se detiene y queda en silencio. **Para la
evaluación no hace falta pasar ningún argumento**: los valores por defecto
son los validados.

### Salida esperada

```
[dock_lidar]: dock_lidar listo | /scan -> /cmd_vel a 20 Hz | v_max 0.30 m/s, w_max 1.50 rad/s, v_fine 0.050, v_dock 0.025 | range_sigma 0.001 m
[dock_lidar]: TF disponible, arrancando
[dock_lidar]: [  0.05s] SEARCH -> APPROACH
[dock_lidar]: [ 11.70s] APPROACH -> DOCK
[dock_lidar]: is_docked = true a los 14.25 s
[dock_lidar]: [ 14.65s] DOCK -> DONE
```

La primera línea resume la configuración efectiva: conviene revisarla antes
de cada corrida.

### Otras formas útiles

```bash
# Ver todos los argumentos con su descripción
ros2 launch terraslam_solucion_dock_lidar solucion.launch.py --show-args

# Telemetría del control cada 0,5 s: distancia a la pared, error lateral,
# error de rumbo, lookahead y velocidad
ros2 launch terraslam_solucion_dock_lidar solucion.launch.py log_level:=debug
#   d=0.371 e=-5.0mm head=+0.00deg L=0.06 v=0.050

# Probar desde otra pose (en la terminal del escenario)
ros2 launch create3_dock_challenge challenge_world.launch.py x:=1.3 y:=0.9 yaw:=1.0

# Create 3 real con el RPLIDAR C1
ros2 launch terraslam_solucion_dock_lidar solucion.launch.py use_sim_time:=false range_sigma:=0.01
```

En RViz: **Add → By topic → `/dock_lidar/markers`** muestra las cajas
detectadas (naranja) y el eje estimado del dock (cian).

---

## Parámetros del launch

Cada argumento del launch es un parámetro del nodo con el mismo nombre. Las
distancias `d_*` se miden **desde la pared detectada, a lo largo de su
normal**, nunca en coordenadas del mundo.

### Entorno e interfaces

| Argumento | Defecto | Descripción |
|---|---|---|
| `use_sim_time` | `true` | Reloj de Gazebo. `false` en el robot real. |
| `scan_topic` | `/scan` | LaserScan de entrada. |
| `cmd_vel_topic` | `/cmd_vel` | Twist de salida. |
| `dock_status_topic` | `/dock_status` | Solo se lee `is_docked`. |
| `odom_frame`, `base_frame` | `odom`, `base_link` | Frames de TF. |
| `publish_markers` | `true` | Markers de depuración para RViz. |
| `control_rate_hz` | `20.0` | Frecuencia del control y de `/cmd_vel`. Mínimo 5 Hz: el Create 3 se detiene si no recibe comandos seguido. |
| `log_level` | `info` | `debug` activa la telemetría del control. |

### Percepción

| Argumento | Defecto | Descripción |
|---|---|---|
| `range_sigma` | `0.001` | Ruido del LiDAR [m]. **Todas las tolerancias del detector escalan con él.** 0,001 en simulación; ~0,01 en el C1 real. |
| `max_scan_range` | `6.0` | Ignora lecturas más lejanas [m]. |

### Velocidades

| Argumento | Defecto | Descripción |
|---|---|---|
| `v_max` | `0.30` | Velocidad lineal máxima [m/s]. Límite del Create 3: 0,46 con `safety_override=full` (el que aplica el escenario). |
| `w_max` | `1.5` | Velocidad angular máxima [rad/s]. |
| `v_fine` | `0.05` | Velocidad en el tramo fino [m/s]. |
| `v_dock` | `0.025` | Velocidad sobre la rampa [m/s]. |
| `v_backoff` | `0.12` | Velocidad de retroceso al reintentar [m/s]. |
| `w_search` | `0.8` | Giro de búsqueda [rad/s]. |

### Ganancias y geometría de la maniobra

| Argumento | Defecto | Descripción |
|---|---|---|
| `k_w` | `3.0` | Ganancia proporcional de rumbo. |
| `k_i` | `2.5` | Ganancia integral de rumbo (solo en el tramo fino). |
| `d_fine` | `0.60` | Inicio del tramo fino [m]. |
| `d_slow` | `0.42` | Desde aquí se exige alineación; si no, retrocede y reintenta [m]. |
| `d_dock` | `0.33` | Inicio del tramo de rampa [m]. |
| `d_floor` | `0.245` | Distancia mínima sin acoplar: más cerca, retrocede [m]. |
| `d_stage` | `0.75` | Punto de preparación sobre el eje [m]. |
| `align_tol_lat` | `0.025` | Error lateral máximo al pasar `d_slow` [m]. |
| `align_tol_head_deg` | `15.0` | Error de rumbo máximo al pasar `d_slow` [°]. |

### Choques (`/hazard_detection`)

| Argumento | Defecto | Descripción |
|---|---|---|
| `hazard_topic` | `/hazard_detection` | `HazardDetectionVector`; solo se usa el tipo `BUMP`. |
| `react_to_bumps` | `true` | Ante un choque: retroceder recto (`ESCAPE`) y reintentar. |
| `bump_escape_s` | `0.8` | Duración del retroceso recto tras un choque [s]. |
| `max_bump_retries` | `3` | Reintentos por choque; después se ignoran, para no quedar en bucle si el contacto es legítimo (rampa del dock). |

### Búsqueda, exploración y final

| Argumento | Defecto | Descripción |
|---|---|---|
| `start_delay_s` | `1.0` | Espera inicial [s]. |
| `search_turn_deg` | `90.0` | Giro en `SEARCH` antes de explorar [°]. |
| `explore_step` | `0.8` | Paso máximo de `EXPLORE` [m]. |
| `explore_timeout_s` | `8.0` | Tiempo máximo por paso de `EXPLORE` [s]. |
| `obstacle_clearance` | `0.45` | `EXPLORE` se detiene si hay algo a menos de esto por delante [m]. |
| `dock_push_s` | `0.4` | Sigue empujando tras `is_docked` para asentar los contactos [s]. |

El nodo **valida los valores al arrancar**: sube `control_rate_hz` a 5 Hz
como mínimo y avisa si `v_max > 0,46`, si `d_floor < 0,23` (tocaría las
cajas), si `v_dock > 0,05` o si las distancias no están en orden
`d_floor < d_dock < d_slow < d_fine`.

**Ajustes típicos:**
- *Más rápido:* `v_max:=0.40 v_fine:=0.07`. Medido en dos poses: unos 2,5 s
  menos (13,1 y 15,4 s) con la misma precisión. Con 16,7 s de media ya se
  está muy por debajo de los 45 s que dan puntaje completo, así que por
  defecto se prefiere el margen.
- *Más preciso o más seguro:* `v_fine:=0.04 d_fine:=0.70`.
- *LiDAR con más ruido:* sube `range_sigma`.

---

## Cómo funciona

Todo está en un solo nodo, [`terraslam_solucion_dock_lidar/dock_lidar.py`](terraslam_solucion_dock_lidar/dock_lidar.py),
organizado en tres capas:

```
 /scan (10 Hz) ──► PERCEPCIÓN ──► ESTIMACIÓN ──► CONTROL (20 Hz) ──► /cmd_vel
                   detector        Kalman en      máquina de
                   geométrico      odom           estados + pure pursuit
 /tf ─────────────────┴───────────────┘                 ▲
 /dock_status (is_docked) ──────────────────────────────┤
 /hazard_detection (BUMP) ──────────────────────────────┘
```

### 1. Percepción: cómo sabe el LiDAR que esas son las cajas

El LiDAR no ve colores ni formas: devuelve 720 distancias por vuelta. El
detector busca en esos puntos una **firma geométrica** que solo pueden
producir las dos cajas, usando exclusivamente sus medidas publicadas en el
reto:

```
                 vista superior (el robot está abajo)

   pared ───────┐        ┌────────┐        ┌───────── pared
                │  caja  │ hueco  │  caja  │
                │  8 cm  │ 9,5 cm │  8 cm  │     ▲
                └────────┘        └────────┘     │ 8 cm (saliente)
                     │◄──── 17,5 cm ───►│         ▼
                              ▲
                         eje del dock
```

**Paso 0 — Puntos en el frame del robot.** Cada rayo `i` se convierte a
`(x, y)` con el ángulo `angle_min + i·angle_increment` y se transforma con el
TF `laser_link → base_link`. Eso resuelve de una vez que el LiDAR está 5 cm
detrás, 1,8 cm a la derecha y **montado girado 180°** (el rayo 0 apunta
hacia atrás). No se usa ningún índice del array como dirección.

**Paso 1 — Paredes candidatas (RANSAC).** Se extraen hasta 6 rectas con
RANSAC secuencial: se toman dos puntos, se cuentan los que quedan a menos de
`max(12 mm, 2,5σ)` de la recta que definen, y la mejor recta (mínimo 12
puntos) se refina con mínimos cuadrados totales (SVD). Se quitan sus puntos
y se repite. Así salen la pared del fondo, las laterales y cualquier otra
superficie plana.

**Paso 2 — Puntos que sobresalen.** Para cada recta se calcula, en cada
punto, su altura `h` sobre la pared (hacia el robot) y su posición `s` a lo
largo de ella. Las caras frontales de las cajas son puntos con
`h ≈ 8 cm` (banda de ±`max(3 cm, 3σ)`).

**Paso 3 — Grupos del ancho de una caja.** Los puntos que sobresalen se
ordenan por `s` y se cortan donde hay un salto mayor a 3,5 cm. Un grupo es
candidato a caja si tiene al menos 2 puntos y su ancho observado no supera
`8 cm + max(1,5 cm, 3σ)`.

**Paso 4 — Validar el par.** Dos grupos solo se aceptan como el marcador si
se cumplen **todas** estas condiciones:

| # | Condición | Qué descarta |
|---|---|---|
| 1 | Centros separados **17,5 cm ± 3,5 cm** | Cualquier par de objetos a otra distancia |
| 2 | **Nada sobresale en el hueco** (h > 2,5 cm entre las cajas) | Un objeto ancho o un bloque único |
| 3 | **Pared visible en el hueco**, o pared a ambos lados si la vista es muy oblicua | Dos objetos flotando delante de nada |
| 4 | **Nada sobresale hasta 30 cm a cada lado** del par | Esquinas, columnas, estructuras más grandes |
| 5 | Ajuste conjunto (paso 5) con **saliente 8 cm ± 2 cm** y residuo < 1 cm | Superficies no paralelas o que no son planas |
| 6 | Anchos reconstruidos (tras triangular bordes) de **8 cm ± 3 cm** | Grupos que coinciden en posición pero no en tamaño |

Todas las tolerancias escalan con `range_sigma`. Si hay varios candidatos,
gana el de mayor puntaje: más puntos, menor error de separación, menor
residuo y cercanía a la estimación anterior del filtro.

**Paso 5 — Ajuste conjunto de dos rectas paralelas.** La pared local (±60 cm
alrededor de las cajas) y las caras frontales se ajustan **juntas**, con una
normal compartida: se centra cada grupo en su propio centroide, se apilan y
se toma el vector singular menor (SVD). La pared aporta longitud (orientación
precisa) y las caras aportan puntos cercanos. La selección es iterativa y
estrecha (dos pasadas: ±10/12 mm y luego ±5/6 mm) para que **no entren los
puntos de las caras laterales** de las cajas, que en vistas oblicuas son
muchos y tienen `h` entre 0 y 8 cm. Este fue un fallo real encontrado en las
pruebas, resuelto así.

Usar la pared *local* y no la pared entera hace que funcione aunque haya
otras estructuras cerca, como la columna junto a las cajas en el montaje
real del laboratorio.

### 2. Triangulación sub-muestra de los bordes

A 1,5 m, dos rayos consecutivos del LiDAR están separados unos 1,3 cm sobre
la pared. Si se toma el último punto que toca la caja como su borde, el
error es de hasta media muestra (~7 mm) y siempre hacia adentro. El detector
lo evita **triangulando**:

```
        rayo k (toca la caja)   bisectriz    rayo k+1 (pasa de largo)
                  \                 |                /
                   \                |               /
     cara frontal ──●───────────────✕              /
                    │               ▲             /
                    │         borde estimado     /
                    │                           ●  (golpea la pared detrás)
```

El borde real está en algún punto entre los rayos `k` y `k+1`. Se toma la
**bisectriz** de los dos rayos y se intersecta con el plano de la cara
frontal. El error queda repartido de forma uniforme alrededor de cero en vez
de sesgado. Si el rayo vecino cae en la cara **lateral** de la caja, esa
cara está exactamente en el borde y da su posición directa.

Con los **cuatro bordes** se calculan los centros de las dos cajas, y el
**eje del dock es la media de ambos centros**: equivale al centro del hueco,
pero promediando cuatro mediciones en vez de dos.

Precisión del detector sobre **un solo scan** con σ = 1 mm, medida con un
ray-caster sobre 400 poses aleatorias:

| Zona | Error lateral p50 / p95 | Error angular p50 / p95 |
|---|---|---|
| Rango de salida (0,65–1,95 m de la pared) | 1,0 / 3,2 mm | 0,011° / 0,031° |
| Cerca del dock (0,26–0,60 m) | 0,36 / 1,1 mm | 0,006° / 0,018° |
| Con ruido de 1 cm (LiDAR real) | 1,5 / 5,8 mm | 0,17° / 0,49° |

Cuesta ~10 ms por scan.

### 3. Estimación: filtro de Kalman en `odom`

El dock no se mueve, así que el estado es estático: el punto de la pared
sobre el eje `W` y el ángulo de la normal `α`, ambos en el frame `odom`.
Cada detección se lleva a `odom` con el TF **del instante del scan**, de
modo que el giro del robot durante el procesamiento no la desplaza.

- **Ruido de medida según la distancia**, calibrado con la tabla anterior:
  `σ_pos = 0,8 mm + 2,5 mm/m · rango + 1,5 · residuo` y
  `σ_ang = 0,02° + 0,03°/m · rango + residuo`. Las lecturas cercanas, que
  son las más precisas, dominan al final.
- **Ruido de proceso según el movimiento**, porque la odometría deriva:
  `P_pos += (1 % · Δs)² + (0,02 m/rad · Δθ)²` y `P_ang += (1 % · Δθ)² + (0,005 rad/m · Δs)²`.
- **Gating:** se descarta una medida a más de `4σ + 2 cm` o `4σ + 2°` del
  estado. Si llegan 5 rechazos seguidos, se reinicia, porque varias medidas
  coherentes que contradicen el estado significan que el estado estaba mal.
- **Memoria:** si el marcador se pierde unos ciclos (oclusión, rango mínimo),
  el robot sigue guiado por la última estimación más la odometría.

### 4. Control en el frame del dock

Todo el control se hace en coordenadas del dock:

```
  d = (p − W) · n     distancia del centro del robot a la pared, sobre la normal
  e = (p − W) · t     error lateral respecto al eje (t ⟂ n)
  θ_pared = α + π     rumbo "mirando a la pared"
```

#### Máquina de estados

```
             ┌──────────┐ sin marcador tras girar ┌─────────┐
  inicio ──► │  SEARCH  │ ──────────────────────► │ EXPLORE │ ─┐
             └────┬─────┘ ◄────────────────────── └────┬────┘  │
                  │ marcador visto                     │       │
                  ▼                                    │       │
    fuera del cono│   ┌─────────┐                      │       │
                  ├──►│  STAGE  │──┐ entra al cono     │       │
                  │   └─────────┘  │                   │       │
                  ▼                ▼                   ▼       │
             ┌──────────────────────────┐   d < d_dock  ┌──────┴─┐
             │         APPROACH         │ ────────────► │  DOCK  │
             └──────────────────────────┘               └───┬────┘
                  ▲      desalineado cerca / d < d_floor    │ is_docked
                  │         ┌──────────┐ ◄──────────────────┤
                  └─────────│ BACKOFF  │                    ▼
                            └──────────┘               ┌────────┐
                                                       │  DONE  │
                                                       └────────┘
```

Además, desde `SEARCH`, `EXPLORE`, `STAGE`, `APPROACH` o `DOCK`, un choque
del parachoques lleva a **`ESCAPE` → `BACKOFF`**, o a `SEARCH` si todavía no
se conoce el dock.

| Estado | Qué hace |
|---|---|
| `SEARCH` | Sin estimación: espera unos scans y gira 90°, por si algo montado en el robot tapa un sector del LiDAR. Con 360° de campo de visión casi nunca hace falta más. |
| `EXPLORE` | Si sigue sin ver el marcador (ocluido, o visto con más de ~65° de inclinación), avanza hasta `explore_step` hacia el **centroide de área del polígono de espacio libre** que ve el LiDAR. Ese centroide es el centro de la zona despejada: alejarse de las paredes abre el ángulo con que se ve el marcador. Se detiene si hay un obstáculo a menos de `obstacle_clearance` por delante. |
| `STAGE` | Si el robot está fuera del **cono de aproximación** (`|e| > 0,55 · (d − 0,30)`: demasiado lateral y cerca de la pared para entrar directo), va primero a un punto sobre el eje a `d_stage`. Entra en `APPROACH` con histéresis (pendiente 0,45). |
| `APPROACH` | Pure pursuit sobre el eje (ver abajo), con la velocidad bajando con `d`. |
| `DOCK` | Tramo de rampa a `v_dock` hasta `is_docked`. |
| `ESCAPE` | El parachoques detectó un choque (desde cualquier estado de maniobra): retrocede recto `bump_escape_s` y pasa a `BACKOFF` si ya conoce el dock, o a `SEARCH` si no. Ver [Reacción a choques](#reacción-a-choques). |
| `BACKOFF` | Si pasa `d_slow` desalineado (más de `align_tol_lat` o `align_tol_head_deg`) o baja de `d_floor`, retrocede corrigiendo el rumbo hasta `d_slow + 0,20 m` y reintenta. |
| `DONE` | Tras `is_docked`, empuja `dock_push_s` más para asentarse sobre los contactos, publica ceros y queda en silencio. |

#### Reacción a choques

`/hazard_detection` es el tópico donde el Create 3 publica sus peligros
físicos: una lista de detecciones, cada una con un **tipo** y, en
`header.frame_id`, el **sensor** que la produjo.

| Tipo | Significado | ¿Se usa? |
|---|---|---|
| `BUMP` (1) | El parachoques tocó algo (`bump_front_center`, `bump_left`, …) | **Sí** |
| `CLIFF` (2) | Los sensores de desnivel no ven piso | No: `safety_override=full` los desactiva y la sala es plana |
| `BACKUP_LIMIT` (0) | Retrocedió más de lo que permite su modo de seguridad | No: desactivado con `full` |
| `WHEEL_DROP` (4), `STALL` (3), `OBJECT_PROXIMITY` (5) | Rueda colgando, ruedas trabadas, IR frontal | No |

Cada choque con cajas, dock o pared cuesta **−10 puntos por corrida**. El
firmware del Create 3 ya tiene un reflejo (`REFLEX_BUMP`) que retrocede un
poco, pero es ciego: no sabe dónde está el dock y la maniobra volvería a
empujar contra lo mismo. Por eso, ante un `BUMP`:

1. **`ESCAPE`:** retrocede **recto** `bump_escape_s` (0,8 s a `v_backoff`).
   Recto porque el parachoques es frontal: girar arrastraría el borde del
   robot contra lo que tocó.
2. **`BACKOFF`:** retrocede sobre el eje, corrigiendo el rumbo, hasta
   `d_slow + 0,20 m`, y reintenta la aproximación con el filtro intacto.
   Si todavía no conocía el dock, vuelve a `SEARCH`.
3. **Límite de reintentos** (`max_bump_retries` = 3): después, los choques se
   ignoran. Así un contacto legítimo, como el del frente del robot con la
   rampa, no puede dejarlo en un bucle infinito.

Se ignoran los choques con el robot ya acoplado, en `DONE` o durante el
propio `ESCAPE`. En las pruebas normales el parachoques **nunca** se activó,
ni siquiera al subir la rampa del dock: la reacción es una red de seguridad,
sobre todo para el robot real.

#### Ley de control: pure pursuit sobre el eje

El robot apunta a un punto del eje situado a una distancia `L` por delante:

```
  θ_deseado = θ_pared + atan2(e, L)
  ω = k_w · (θ_deseado − θ) + k_i · ∫(θ_deseado − θ) dt
  L = clip(0,35 · (d − 0,26), 0,05, 0,50)          en DOCK: L = max(L, 0,05 + 3·(d_dock − d))
```

Se corrigen el error lateral y el angular **a la vez y de forma continua**,
sin alinear primero y avanzar después: cualquier deslizamiento durante la
aproximación se corrige solo.

#### Por qué es tan preciso (el razonamiento, medido en simulación)

Para una aproximación con esta ley, el error lateral decae como
**e(s) ≈ e₀ · exp(−s / L)** con la distancia recorrida `s`, y el rumbo
residual al llegar es **≈ e / L**. De ahí salen las decisiones:

1. **`L` corto desde lejos y un tramo fino largo** (desde `d_fine` = 0,60 m
   a 5 cm/s). El lateral se consume unas 5 constantes de tiempo antes de la
   rampa. Con la primera versión (`L` mínimo de 10 cm y tramo lento corto) el
   robot llegaba con ~1,4 mm y ~1,2° de error. Este cambio los bajó a 0,1 mm
   y 0,5°.
2. **Término integral con fuga en el tramo fino.** A 2–5 cm/s, un error de
   rumbo de décimas de grado pide una diferencia de ~1 mm/s entre las ruedas,
   que el robot no ejecuta por fricción y por la rampa. La telemetría
   mostraba un error persistente de ~0,3° que la parte proporcional no
   vencía; la integral lo elimina. La fuga (×0,97 por ciclo) y el límite
   (±0,02 rad·s) evitan el sobreimpulso.
3. **`L` continuo en `d`.** Un intento de subir `L` de golpe a 0,15 m al
   entrar a la rampa produjo un salto en el rumbo deseado, sobreimpulso de la
   integral y 2,2 mm de error lateral. Por eso en `DOCK` crece de forma
   continua: el término `e/L` casi desaparece sin transitorios.

Velocidad lineal: `v = clip(0,7·(d − d_fine) + v_fine, v_fine, v_max)` lejos,
`v_fine` en el tramo fino y `v_dock` en la rampa. Se multiplica por
`cos⁴(error de rumbo)` y se anula si el error supera 1 rad: primero gira y
luego avanza.

---

## Condiciones consideradas

### Reglas del reto

| Condición | Cómo se cumple |
|---|---|
| Solo LiDAR para percibir | La única percepción es `/scan`. `/tf` se usa para el montaje del LiDAR y la odometría, ambos permitidos. |
| Interfaces prohibidas | No se usan la acción `/dock` ni las demás acciones de iRobot, `/ir_opcode`, `/ir_intensity`, el ground truth, `/gazebo/*` ni el frame `std_dock_link` (que en este escenario es incorrecto). |
| `/dock_status` | Solo el campo `is_docked`. **No** `dock_visible`, que sale de los sensores IR. |
| `/hazard_detection` | Permitido. Solo el tipo `BUMP`, para reaccionar a choques. |
| `/imu`, `/battery_state` | Permitidos pero no usados: el rumbo sale del TF de odometría, y la batería no influye en corridas de ~17 s. |
| Sin hardcodeo | No hay poses, distancias a la pared ni secuencias de movimiento fijas. Las únicas constantes geométricas son las **medidas del marcador** (8 cm, 8 cm, 9,5 cm), que el README del reto indica usar para reconocer la firma. |
| No modificar el escenario | `create3_dock_challenge` queda intacto; la solución es un paquete aparte. |
| Comando único | `ros2 launch terraslam_solucion_dock_lidar solucion.launch.py`, sin argumentos obligatorios. |
| 180 s por corrida | El peor caso medido es 18,9 s. |

### Sensor y robot

| Condición | Cómo se trata |
|---|---|
| LiDAR 5 cm detrás, 1,8 cm a la derecha, girado 180° | Todo pasa por TF a `base_link`; nunca se asume que `ranges[0]` apunta al frente. |
| Plano de escaneo a 17,75 cm | Corta las cajas (13–25 cm) a media altura. Se verificó en el scan real que **el dock no aparece** en ese plano, así que no interfiere. |
| Rango mínimo 0,15 m | Acoplado, las cajas quedan a ~0,24 m del LiDAR: siempre visibles. Si se pierden, el filtro mantiene la estimación. |
| Ruido σ = 1 mm | Las tolerancias escalan con `range_sigma`; probado offline hasta 1 cm. |
| Resolución de 0,5° (~1,3 cm entre puntos a 1,5 m) | Triangulación sub-muestra de los bordes. |
| Vistas oblicuas (hasta 58° en el rango del jurado) | Selección iterativa que excluye las caras laterales; detecta hasta ~65°. Más allá, `EXPLORE`. |
| Oclusiones y estructuras cercanas (columna del laboratorio) | Pared ajustada localmente; validaciones 2–4 del par; `EXPLORE`. |
| El Create 3 se detiene sin comandos continuos | `/cmd_vel` a 20 Hz (mínimo forzado de 5 Hz). |
| Velocidad máxima 0,46 m/s (`safety_override=full`) | `v_max` = 0,30 por defecto, con aviso si se pasa de 0,46. |

### Acoplamiento

| Condición | Cómo se trata |
|---|---|
| `is_docked` exige receptor–emisor < 7,5 cm, ±6° sobre el eje y ±6° de rumbo | El control lleva ambos errores a ~0 antes de la rampa; peor caso medido 0,24°. |
| El centro debe llegar a menos de ~26,8 cm de la pared | El robot avanza a `v_dock` hasta que `is_docked` se activa, sin una distancia objetivo fija. |
| Tocar cajas, dock o pared penaliza (−10) | Con el frente circular del robot, el contacto con las cajas ocurre a `d` < 22,5 cm; `d_floor` = 24,5 cm fuerza un retroceso antes. Si aun así el parachoques detecta un choque, `ESCAPE` + `BACKOFF` + reintento. 0 choques en todas las pruebas. |
| La rampa hace rebotar si se entra rápido o torcido | 2,5 cm/s en la rampa, y `BACKOFF` si llega desalineado. |
| Quedarse corto es el fallo más común | No se para por distancia sino por `is_docked`, y luego empuja 0,4 s más. |
| Puntaje de tiempo completo hasta 45 s | Media de 16,7 s. |

---

## Validación

Las pruebas usan scripts externos al paquete (el ground truth solo se lee
ahí, para medir): un ray-caster 2D para el detector y un evaluador que lanza
Gazebo en modo headless desde cada pose, corre `solucion.launch.py` y mide
tiempo, error lateral, error angular y choques (`/hazard_detection`).

**Validación final:** 20 corridas desde poses aleatorias de
x ∈ [0; 1,3], y ∈ [−0,9; 0,9], yaw ∈ [−π; π], incluyendo salidas de espaldas
al dock.

| Métrica | Media | Peor caso |
|---|---|---|
| Acoplamientos (`is_docked`, sin choques) | 20/20 | — |
| Tiempo hasta `is_docked` | 16,7 s | 18,9 s |
| Error lateral | 0,10 mm | 0,4 mm |
| Error angular | 0,12° | 0,24° |

Fuera del rango de evaluación, pegado a una pared lateral (y = ±1,5–1,9),
también acopla gracias a `EXPLORE`, en 23–42 s.

**Reacción a choques.** Como en las corridas normales el parachoques nunca
se activa, se probó inyectando un `BUMP` falso en `/hazard_detection` a mitad
de la aproximación, en distintos momentos (lejos, en el tramo fino). En todos
los casos el robot pasó a `ESCAPE`, luego a `BACKOFF`, reintentó y **acopló
igual**, con la misma precisión (≤ 0,2 mm y ≤ 0,06°), en 17–27 s. Las
corridas sin choques dan los mismos resultados que antes de agregar la
reacción.

---

## Limitaciones conocidas

- **Distorsión por movimiento.** Gazebo entrega cada scan como si fuera
  instantáneo. En el robot real, girar rápido mientras el LiDAR barre
  desplaza los puntos; el filtro lo amortigua, pero para la final convendría
  compensarlo o girar más despacio.
- **Postes del RRBOT.** El montaje real tiene postes metálicos alrededor del
  LiDAR que tapan sectores estrechos. El giro de `SEARCH` y el filtro lo
  cubren, pero hay que validarlo en el laboratorio.
- **Vistas muy oblicuas.** Desde poses fuera del rango de evaluación y
  pegadas a una pared lateral, el marcador se ve demasiado de canto y el
  robot tiene que explorar antes de detectarlo: acopla igual, pero en 23–42 s
  en vez de ~17 s.
- **Choques reales.** La reacción a choques se validó con choques
  inyectados, no con colisiones físicas (en simulación la solución no choca).
  En un choque real el firmware también ejecuta su reflejo `REFLEX_BUMP`, que
  puede retrasar unos segundos la maniobra. En una de las pruebas, el
  `BACKOFF` tardó 10 s en vez de ~1 s sin causa confirmada, aunque el robot
  acopló igual.
- **Distancias largas.** Más allá de ~3 m de la pared, cada caja recibe muy
  pocos rayos y la detección se vuelve intermitente. Queda fuera del rango
  de evaluación.

---

## Estructura

```
terraslam_solucion_dock_lidar/
├── package.xml
├── setup.py, setup.cfg
├── README.md
├── launch/
│   └── solucion.launch.py      ← comando único, todos los parámetros
└── terraslam_solucion_dock_lidar/
    └── dock_lidar.py           ← el nodo: detector + Kalman + control
```

Licencia Apache-2.0.
