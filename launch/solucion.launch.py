"""Comando unico de la solucion.

    ros2 launch terraslam_solucion_dock_lidar solucion.launch.py

Un solo nodo. Defaults del RBOT fisico (RPLIDAR A1), basados en la corrida
exitosa de 0.14/0.05 m/s. Limites absolutos: 0.20 m/s y 0.50 rad/s.

Ver todos los argumentos con su descripcion:

    ros2 launch terraslam_solucion_dock_lidar solucion.launch.py --show-args

Ejemplos:

    # ver la telemetria del control (d, e, rumbo) cada 0.5 s
    ros2 launch terraslam_solucion_dock_lidar solucion.launch.py log_level:=debug

    # Comprobar percepcion y decisiones SIN mover el robot
    ros2 launch terraslam_solucion_dock_lidar solucion.launch.py \\
        dry_run:=true log_level:=debug

    # Simulacion (solo cuando se trabaja con Gazebo)
    ros2 launch terraslam_solucion_dock_lidar solucion.launch.py \\
        use_sim_time:=true range_sigma:=0.001
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# (nombre, valor por defecto, tipo, descripcion). Cada fila es un argumento
# del launch y un parametro del nodo con el mismo nombre. Los defaults deben
# coincidir con los de dock_lidar.py.
PARAMS = [
    # --- Entorno
    ('use_sim_time', 'false', bool,
     'false en el RBOT fisico; true exclusivamente con Gazebo.'),
    ('dry_run', 'false', bool,
     'true evalua el control publicando solo comandos cero. No mueve el robot.'),

    # --- Interfaces
    ('scan_topic', '/scan', str, 'LaserScan de entrada.'),
    ('cmd_vel_topic', '/cmd_vel', str, 'Twist de salida hacia el robot.'),
    ('dock_status_topic', '/dock_status', str,
     'DockStatus; solo se lee is_docked para saber cuando parar.'),
    ('odom_frame', 'odom', str, 'Frame fijo de odometria.'),
    ('base_frame', 'base_link', str, 'Frame del robot.'),
    ('publish_markers', 'true', bool,
     'Publicar /dock_lidar/markers (cajas y eje del dock) para RViz.'),
    ('control_rate_hz', '20.0', float,
     'Frecuencia del lazo de control y de /cmd_vel [Hz]. El Create 3 se '
     'detiene si no recibe comandos seguido: minimo 5 Hz.'),

    # --- Percepcion
    ('range_sigma', '0.01', float,
     'Ruido del LiDAR [m]; todas las tolerancias del detector escalan con '
     'el. 0.001 en simulacion, 0.01 como punto de partida del A1 real.'),
    ('max_scan_range', '6.0', float,
     'Ignorar lecturas mas lejanas [m]. Mas lejos las cajas reciben muy '
     'pocos rayos y solo aportan ruido.'),

    # --- Velocidades
    ('v_max', '0.14', float,
     'Velocidad lineal maxima [m/s]. Se limita siempre a 0.20 por Kalman.'),
    ('w_max', '0.4', float,
     'Velocidad angular maxima [rad/s]. Se limita siempre a 0.50 por Kalman.'),
    ('v_fine', '0.05', float,
     'Velocidad en el tramo fino, desde d_fine hasta la rampa [m/s].'),
    ('v_dock', '0.035', float,
     'Velocidad sobre la rampa del dock [m/s]. Rapido = rebote.'),
    ('v_seat', '0.01', float,
     'Velocidad de asentamiento recto, limitada a 0.01 m/s.'),
    ('v_backoff', '0.12', float, 'Velocidad de retroceso al reintentar [m/s].'),
    ('w_search', '0.3', float, 'Velocidad de giro en SEARCH [rad/s].'),

    # --- Ganancias del control de rumbo
    ('k_w', '1.0', float, 'Ganancia proporcional de rumbo.'),
    ('k_i', '0.0', float,
     'Ganancia integral de rumbo (solo en el tramo fino): vence la friccion '
     'que deja un error de rumbo residual a baja velocidad.'),

    # --- Geometria de la maniobra (distancias a la pared DETECTADA)
    ('d_fine', '0.75', float, 'Inicio del tramo fino de precision [m].'),
    ('d_slow', '0.42', float,
     'Desde aqui se exige alineacion; si no, retrocede y reintenta [m].'),
    ('d_dock', '0.33', float, 'Inicio del tramo de rampa a v_dock [m].'),
    ('d_floor', '0.231', float,
     'Distancia de parada para esperar dock_status, sin retroceso inmediato [m].'),
    ('d_stage', '0.75', float,
     'Distancia del punto de preparacion sobre el eje (STAGE) [m].'),
    ('align_tol_lat', '0.025', float,
     'Error lateral maximo al pasar d_slow [m].'),
    ('align_tol_head_deg', '15.0', float,
     'Error de rumbo maximo al pasar d_slow [grados].'),

    # --- Busqueda y exploracion
    ('start_delay_s', '1.0', float, 'Espera inicial antes de moverse [s].'),
    ('search_turn_deg', '90.0', float,
     'Giro en SEARCH antes de pasar a EXPLORE [grados].'),
    ('explore_step', '0.8', float,
     'Paso maximo hacia el centro del espacio libre en EXPLORE [m].'),
    ('explore_timeout_s', '8.0', float,
     'Tiempo maximo por paso de EXPLORE [s].'),
    ('obstacle_clearance', '0.45', float,
     'EXPLORE se detiene si hay algo a menos de esto por delante [m].'),

    # --- Choques (/hazard_detection)
    ('hazard_topic', '/hazard_detection', str,
     'HazardDetectionVector; solo se usa el tipo BUMP (parachoques).'),
    ('react_to_bumps', 'true', bool,
     'Ante un choque: retroceder recto (ESCAPE) y reintentar la '
     'aproximacion. Cada choque cuesta -10 puntos por corrida.'),
    ('bump_escape_s', '0.8', float,
     'Duracion del retroceso recto tras un choque [s].'),
    ('max_bump_retries', '3', int,
     'Reintentos por choque; despues se detiene hasta relanzar.'),

    # --- Final
    ('dock_push_s', '0.0', float,
     'Obsoleto e ignorado: la confirmacion se realiza parado.'),
    ('dock_confirm_s', '2.0', float,
     'Tiempo de true consecutivo y al menos tres mensajes, parado [s].'),
    ('dock_wait_s', '3.0', float,
     'Espera parado antes del siguiente pulso de asentamiento [s].'),
    ('seat_step', '0.003', float,
     'Avance maximo por pulso final, medido con odom [m].'),
    ('max_seat_steps', '0', int,
     'Pulsos finales experimentales (0 a 3), desactivados por defecto en RBOT.'),
    ('seat_min_distance', '0.230', float,
     'Distancia minima permitida para seguir asentando [m].'),
    ('seat_timeout_s', '0.6', float,
     'Duracion maxima por pulso; tambien se limita por seat_step/v_seat [s].'),
    ('seat_tol_lat', '0.006', float,
     'Error lateral maximo para asentar recto [m].'),
    ('seat_tol_head_deg', '3.0', float,
     'Error angular maximo para asentar recto [grados].'),
    ('dock_timeout_s', '12.0', float,
     'Tiempo maximo avanzando en DOCK antes de esperar parado [s].'),

    # --- Barrido de rumbo en el contacto (estado SWEEP)
    ('dock_sweep', 'true', bool,
     'Si llega a la distancia de acople sin is_docked: girar en el sitio por '
     'pulsos (+-1.5, +-3 ... grados) y parar quieto tras cada uno. En el RBOT '
     'el dock esta girado ~3 grados respecto a las cajas.'),
    ('sweep_max_deg', '6.0', float, 'Amplitud maxima del barrido [grados].'),
    ('sweep_step_deg', '1.5', float, 'Paso entre objetivos del barrido [grados].'),
    ('sweep_w', '0.3', float,
     'Giro de cada pulso [rad/s]. >=0.26: por debajo las ruedas no llegan al '
     'minimo de 0.03 m/s del firmware.'),
    ('sweep_hold_s', '1.2', float,
     'Pausa quieto tras cada pulso; dock_status llega a 1 Hz [s].'),
    ('sweep_tol_lat', '0.02', float,
     'Error lateral maximo para iniciar el barrido [m].'),
    ('sweep_timeout_s', '25.0', float, 'Duracion maxima del barrido [s].'),
    ('max_dock_retries', '2', int,
     'Reintentos totales, ademas del intento inicial; despues STOPPED.'),
    ('retry_timeout_s', '12.0', float,
     'Tiempo maximo de una maniobra de recuperacion [s].'),
    ('scan_timeout_s', '0.75', float, 'Edad maxima del scan [s].'),
    ('tf_timeout_s', '0.75', float, 'Edad maxima del TF de odometria [s].'),
    ('detection_timeout_s', '3.0', float,
     'Tiempo maximo sin una deteccion aceptada del marcador [s].'),
    ('dock_status_timeout_s', '2.5', float,
     'Edad maxima del estado de acoplamiento [s].'),
    ('startup_timeout_s', '15.0', float,
     'Espera maxima de LiDAR, TF y dock_status al arrancar [s].'),
    ('run_timeout_s', '180.0', float, 'Duracion maxima de la corrida [s].'),
]


def generate_launch_description():
    args = [DeclareLaunchArgument(name, default_value=default,
                                  description=desc)
            for name, default, _, desc in PARAMS]
    args.append(DeclareLaunchArgument(
        'log_level', default_value='info',
        choices=['debug', 'info', 'warn', 'error'],
        description='debug muestra la telemetria del control cada 0.5 s.'))

    params = {name: ParameterValue(LaunchConfiguration(name), value_type=typ)
              for name, _, typ, _ in PARAMS}

    node = Node(
        package='terraslam_solucion_dock_lidar',
        executable='dock_lidar',
        name='dock_lidar',
        output='screen',
        emulate_tty=True,
        parameters=[params],
        ros_arguments=['--log-level',
                       ['dock_lidar:=', LaunchConfiguration('log_level')]],
    )
    return LaunchDescription(args + [node])
