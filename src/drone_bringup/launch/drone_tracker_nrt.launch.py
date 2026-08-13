"""
drone_tracker_nrt.launch.py

Pipeline completo de deteccion y localizacion en tiempo real (variante NRT).

Arquitectura:
    rtsp_publisher  ─┐
                     ├─> yolo_detection -> georeferencer_nrt -> map_server -> web dashboard
    mqtt_telemetry  ─┘

Diferencias respecto a drone_tracker_live.launch.py:
  - Video via RTSP (MediaMTX) en lugar de capturadora USB.
  - Georeferencer NRT: solo geolocaliza cuando el dron lleva >= stability_window_s
    segundos estable en posicion y angulos. Sin sincronizacion por timestamp.
  - Opcionalmente lanza un dashboard web externo (ver web_app_path).

Prerequisitos:
  - Broker MQTT y servidor MediaMTX accesibles (ver README).
  - Dron transmitiendo RTMP a MediaMTX.

Uso basico:
    ros2 launch drone_bringup drone_tracker_nrt.launch.py

Con URL RTSP explicita (si auto-descubrimiento falla):
    ros2 launch drone_bringup drone_tracker_nrt.launch.py \\
        rtsp_url:=rtsp://127.0.0.1:8554/live/<stream-id>

Argumentos disponibles:
    rtsp_url            URL RTSP completa. Si vacio, auto-descubre desde MediaMTX.
    mediamtx_api        URL base de la API de MediaMTX (default: http://127.0.0.1:9997).
    broker_host         Host del broker MQTT EMQX (default: 127.0.0.1).
    broker_port         Puerto del broker MQTT (default: 1883).
    aircraft_sn         Numero de serie del Mavic 3T.
    focal_len           Focal length de la camara termica en mm (default: 40.0).
    conf_threshold      Umbral de confianza YOLO (default: 0.5).
    iou_threshold       Umbral IoU para NMS (default: 0.45).
    use_half            FP16 en YOLO: true/false (default: false).
    fov_horizontal      FOV horizontal de la camara en grados (default: 50.5).
    max_distance        Distancia maxima GPS aceptable en metros (default: 1000.0).
    min_distance        Distancia minima GPS aceptable en metros (default: 10.0).
    stability_window_s  Segundos de estabilidad requeridos para geolocalizacion (default: 4.0).
    stability_pos_std_m Max std de posicion en metros para considerar estable (default: 2.0).
    stability_angle_std_deg Max std de angulos en grados (default: 1.0).
    use_kalman          Activar filtro de Kalman: true/false (default: true).
    keep_history        Guardar historial de trayectorias: true/false (default: false).
    output_dir          Directorio de salida para JSON e imagen (default: /tmp/drone_map_data).
    web_app_path        Ruta al dashboard web opcional. Si vacio, no se lanza.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_DEFAULT_AIRCRAFT_SN = ''

# El dashboard web es un componente opcional externo a este workspace. Si
# web_app_path queda vacio, el pipeline corre sin el y el map_server sigue
# exportando los resultados a output_dir.
_DEFAULT_WEB_APP = ''


def _launch_nodes(context, *args, **kwargs):
    rtsp_url = LaunchConfiguration('rtsp_url').perform(context)
    mediamtx_api = LaunchConfiguration('mediamtx_api').perform(context)
    broker_host = LaunchConfiguration('broker_host').perform(context)
    broker_port = LaunchConfiguration('broker_port').perform(context)
    aircraft_sn = LaunchConfiguration('aircraft_sn').perform(context)
    focal_len = LaunchConfiguration('focal_len').perform(context)
    conf_threshold = LaunchConfiguration('conf_threshold').perform(context)
    iou_threshold = LaunchConfiguration('iou_threshold').perform(context)
    use_half = LaunchConfiguration('use_half').perform(context)
    fov_horizontal = LaunchConfiguration('fov_horizontal').perform(context)
    max_distance = LaunchConfiguration('max_distance').perform(context)
    min_distance = LaunchConfiguration('min_distance').perform(context)
    stability_window_s = LaunchConfiguration('stability_window_s').perform(context)
    stability_pos_std_m = LaunchConfiguration('stability_pos_std_m').perform(context)
    stability_angle_std_deg = LaunchConfiguration('stability_angle_std_deg').perform(context)
    use_kalman = LaunchConfiguration('use_kalman').perform(context)
    keep_history = LaunchConfiguration('keep_history').perform(context)
    output_dir = LaunchConfiguration('output_dir').perform(context)
    web_app_path = LaunchConfiguration('web_app_path').perform(context)

    node_rtsp_publisher = Node(
        package='camera_publisher_node',
        executable='rtsp_publisher',
        name='rtsp_publisher',
        output='screen',
        parameters=[{
            'rtsp_url':      rtsp_url,
            'mediamtx_api':  mediamtx_api,
            'publish_rate':  30,
            'use_compression': False,
        }],
    )

    node_mqtt_telemetry = Node(
        package='mqtt_telemetry_node',
        executable='mqtt_telemetry',
        name='mqtt_telemetry',
        output='screen',
        parameters=[{
            'broker_host': broker_host,
            'broker_port': int(broker_port),
            # Credenciales vacias a proposito: el nodo las toma de las variables
            # de entorno DJI_MQTT_USERNAME / DJI_MQTT_PASSWORD.
            'username':    '',
            'password':    '',
            'aircraft_sn': aircraft_sn,
            'focal_len':   float(focal_len),
        }],
    )

    node_yolo_detector = Node(
        package='yolo_detection_node',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'conf_threshold':   float(conf_threshold),
            'iou_threshold':    float(iou_threshold),
            'use_half':         use_half.lower() == 'true',
            'max_frame_age_ms': 200.0,
        }],
    )

    node_georeferencer_nrt = Node(
        package='georeferencing_node',
        executable='georeferencer_nrt',
        name='georeferencer_nrt',
        output='screen',
        parameters=[{
            'fov_horizontal':           float(fov_horizontal),
            'max_person_distance':      float(max_distance),
            'min_person_distance':      float(min_distance),
            'stability_window_s':       float(stability_window_s),
            'stability_pos_std_m':      float(stability_pos_std_m),
            'stability_angle_std_deg':  float(stability_angle_std_deg),
            'use_kalman':               use_kalman.lower() == 'true',
        }],
    )

    node_map_server = Node(
        package='map_server_node',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'output_dir':   output_dir,
            'keep_history': keep_history.lower() == 'true',
            'update_rate':  10,
        }],
    )

    actions = [
        LogInfo(msg='[drone_tracker_nrt] Iniciando pipeline NRT...'),
        LogInfo(msg=f'[drone_tracker_nrt] RTSP: {rtsp_url or "(auto-descubrimiento)"}'),
        LogInfo(msg=f'[drone_tracker_nrt] MQTT: {broker_host}:{broker_port}'),
        LogInfo(msg=f'[drone_tracker_nrt] Estabilidad: {stability_window_s}s / '
                    f'{stability_pos_std_m}m / {stability_angle_std_deg}deg'),
        node_rtsp_publisher,
        node_mqtt_telemetry,
        node_yolo_detector,
        node_georeferencer_nrt,
        node_map_server,
    ]

    if web_app_path:
        actions.append(LogInfo(msg='[drone_tracker_nrt] Dashboard: http://localhost:5000'))
        actions.append(ExecuteProcess(
            cmd=['/usr/bin/python3', web_app_path],
            name='web_dashboard',
            output='screen',
            cwd=os.path.dirname(web_app_path),
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'rtsp_url', default_value='',
            description='URL RTSP. Si vacio, auto-descubre desde MediaMTX.',
        ),
        DeclareLaunchArgument(
            'mediamtx_api', default_value='http://127.0.0.1:9997',
            description='URL base de la API de MediaMTX.',
        ),
        DeclareLaunchArgument(
            'broker_host', default_value='127.0.0.1',
            description='Host del broker MQTT EMQX.',
        ),
        DeclareLaunchArgument(
            'broker_port', default_value='1883',
            description='Puerto del broker MQTT.',
        ),
        DeclareLaunchArgument(
            'aircraft_sn', default_value=_DEFAULT_AIRCRAFT_SN,
            description='Numero de serie del Mavic 3T.',
        ),
        DeclareLaunchArgument(
            'focal_len', default_value='40.0',
            description='Focal length de la camara termica en mm.',
        ),
        DeclareLaunchArgument(
            'conf_threshold', default_value='0.5',
            description='Umbral de confianza YOLO (0.0 - 1.0).',
        ),
        DeclareLaunchArgument(
            'iou_threshold', default_value='0.45',
            description='Umbral IoU para NMS (0.0 - 1.0).',
        ),
        DeclareLaunchArgument(
            'use_half', default_value='false',
            description='Inferencia FP16 en GPU (true/false).',
        ),
        DeclareLaunchArgument(
            'fov_horizontal', default_value='50.5',
            description='FOV horizontal de la camara termica en grados.',
        ),
        DeclareLaunchArgument(
            'max_distance', default_value='1000.0',
            description='Distancia maxima aceptable en metros.',
        ),
        DeclareLaunchArgument(
            'min_distance', default_value='10.0',
            description='Distancia minima aceptable en metros.',
        ),
        DeclareLaunchArgument(
            'stability_window_s', default_value='4.0',
            description='Segundos de estabilidad requeridos para geolocalizacion.',
        ),
        DeclareLaunchArgument(
            'stability_pos_std_m', default_value='2.0',
            description='Max desv. estandar de posicion en metros.',
        ),
        DeclareLaunchArgument(
            'stability_angle_std_deg', default_value='1.0',
            description='Max desv. estandar de angulos en grados.',
        ),
        DeclareLaunchArgument(
            'use_kalman', default_value='true',
            description='Activar filtro de Kalman (true/false).',
        ),
        DeclareLaunchArgument(
            'keep_history', default_value='false',
            description='Guardar historial de posiciones (true/false).',
        ),
        DeclareLaunchArgument(
            'output_dir', default_value='/tmp/drone_map_data',
            description='Directorio de salida para JSON e imagen.',
        ),
        DeclareLaunchArgument(
            'web_app_path', default_value=_DEFAULT_WEB_APP,
            description='Ruta al dashboard web opcional. Si vacio, no se lanza.',
        ),

        OpaqueFunction(function=_launch_nodes),
    ])
