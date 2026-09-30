"""Comando unico de la solucion.

    ros2 launch terraslam_solucion_dock_lidar solucion.launch.py

Se lanza con el escenario del reto ya corriendo en otra terminal. Los valores
por defecto son los validados en simulacion (20/20 acoplamientos): para la
evaluacion no hace falta pasar ningun argumento.

Ver todos los argumentos con su descripcion:

    ros2 launch terraslam_solucion_dock_lidar solucion.launch.py --show-args

Ejemplos:

    # ver la telemetria del control (d, e, rumbo) cada 0.5 s
    ros2 launch terraslam_solucion_dock_lidar solucion.launch.py log_level:=debug

    # Create 3 real con el RPLIDAR C1
    ros2 launch terraslam_solucion_dock_lidar solucion.launch.py \\
        use_sim_time:=false range_sigma:=0.01

    # aproximacion mas agresiva (menos tiempo, algo menos de margen)
    ros2 launch terraslam_solucion_dock_lidar solucion.launch.py v_max:=0.40 v_fine:=0.07
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
    ('use_sim_time', 'true', bool,
     'Reloj de Gazebo. false en el Create 3 real.'),

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
    ('range_sigma', '0.001', float,
     'Ruido del LiDAR [m]; todas las tolerancias del detector escalan con '
     'el. 0.001 en simulacion, ~0.01 en el RPLIDAR C1 real.'),
    ('max_scan_range', '6.0', float,
     'Ignorar lecturas mas lejanas [m]. Mas lejos las cajas reciben muy '
     'pocos rayos y solo aportan ruido.'),

    # --- Velocidades
    ('v_max', '0.30', float,
     'Velocidad lineal maxima [m/s]. Limite del Create 3: 0.46 con '
     'safety_override=full, 0.306 en los otros modos.'),
    ('w_max', '1.5', float, 'Velocidad angular maxima [rad/s].'),
    ('v_fine', '0.05', float,
     'Velocidad en el tramo fino, desde d_fine hasta la rampa [m/s].'),
    ('v_dock', '0.025', float,
     'Velocidad sobre la rampa del dock [m/s]. Rapido = rebote.'),
    ('v_backoff', '0.12', float, 'Velocidad de retroceso al reintentar [m/s].'),
    ('w_search', '0.8', float, 'Velocidad de giro en SEARCH [rad/s].'),

    # --- Ganancias del control de rumbo
    ('k_w', '3.0', float, 'Ganancia proporcional de rumbo.'),
    ('k_i', '2.5', float,
     'Ganancia integral de rumbo (solo en el tramo fino): vence la friccion '
     'que deja un error de rumbo residual a baja velocidad.'),

    # --- Geometria de la maniobra (distancias a la pared DETECTADA)
    ('d_fine', '0.60', float, 'Inicio del tramo fino de precision [m].'),
    ('d_slow', '0.42', float,
     'Desde aqui se exige alineacion; si no, retrocede y reintenta [m].'),
    ('d_dock', '0.33', float, 'Inicio del tramo de rampa a v_dock [m].'),
    ('d_floor', '0.245', float,
     'Distancia minima a la pared sin acoplar: mas cerca retrocede [m]. '
     'Por debajo de ~0.225 el robot toca las cajas.'),
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
     'Reintentos por choque; despues se ignoran para no quedar en bucle '
     'si el contacto es legitimo (rampa del dock).'),

    # --- Final
    ('dock_push_s', '0.4', float,
     'Seguir empujando tras is_docked, para asentar los contactos [s].'),
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
