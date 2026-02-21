#============Aero Robotics Innovations===============
# cotexai_perception.launch.py
# - RealSense splitter + Isaac ROS Visual SLAM + nvblox (+ optional CortexAI manager)
# - Includes: nvblox lightweight preset to reduce CUDA OOM risk
# - Includes: LaunchArguments for IMU fusion / ground constraint with correct bool conversion
#   02.19.2026
#
# Key fixes vs previous version:
#  - Fix nvblox parameters list: parameters=[nvblox_light_params] (avoid TypeError: unhashable type: 'dict')
#  - Add bool LaunchArguments and pass as ParameterValue(..., bool)
#  - Align nvblox pose/map-clearing frame with base_frame (default: camera_link)
#
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.actions import Node


def generate_launch_description():
    # -----------------------------
    # Launch arguments / configs
    # -----------------------------
    camera_ns = LaunchConfiguration('camera_ns')
    use_one_container = LaunchConfiguration('use_one_container')

    # Plugin names (Isaac ROS release-dependent)
    vslam_plugin = LaunchConfiguration('vslam_plugin')
    nvblox_plugin = LaunchConfiguration('nvblox_plugin')

    # Frames
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    imu_frame = LaunchConfiguration('imu_frame')

    # Bool options (strings in launch -> enforce bool via ParameterValue)
    enable_imu_fusion = LaunchConfiguration('enable_imu_fusion')
    enable_ground_constraint_in_odometry = LaunchConfiguration('enable_ground_constraint_in_odometry')
    publish_map_to_odom_tf = LaunchConfiguration('publish_map_to_odom_tf')
    publish_odom_to_base_tf = LaunchConfiguration('publish_odom_to_base_tf')

    # topic helper (LaunchConfiguration-safe concatenation)
    def ns_topic(tail: str):
        # result: "/<camera_ns>/<tail>"
        return ['/', camera_ns, '/', tail.lstrip('/')]

    declare_args = [
        DeclareLaunchArgument('camera_ns', default_value='camera',
                             description='Namespace for RealSense topics (e.g., camera).'),
        DeclareLaunchArgument('use_one_container', default_value='true',
                             description='If true, run splitter+vslam+nvblox+manager in ONE container.'),

        # Plugins
        DeclareLaunchArgument('vslam_plugin',
                             default_value='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                             description='Composable plugin name for isaac_ros_visual_slam.'),
        DeclareLaunchArgument('nvblox_plugin',
                             default_value='nvblox::NvbloxNode',
                             description='Composable plugin name for nvblox_ros.'),

        # Frames
        DeclareLaunchArgument('map_frame', default_value='map',
                             description='VSLAM/Nvblox map frame.'),
        DeclareLaunchArgument('odom_frame', default_value='odom',
                             description='VSLAM/Nvblox odom frame.'),
        # NOTE: default to camera_link to avoid missing base_link TF early on.
        DeclareLaunchArgument('base_frame', default_value='camera_link',
                             description='Robot base frame. Change to base_link once TF is provided.'),
        DeclareLaunchArgument('imu_frame', default_value='camera_gyro_optical_frame',
                             description='IMU frame used by VSLAM (RealSense gyro frame).'),

        # Bool options
        DeclareLaunchArgument('enable_imu_fusion', default_value='true',
                             description='Enable IMU fusion in cuVSLAM.'),
        DeclareLaunchArgument('enable_ground_constraint_in_odometry', default_value='false',
                             description='If true, constrain odometry to ground plane (2D/ Nav2-friendly).'),
        DeclareLaunchArgument('publish_map_to_odom_tf', default_value='true',
                             description='Publish TF map->odom from VSLAM.'),
        DeclareLaunchArgument('publish_odom_to_base_tf', default_value='true',
                             description='Publish TF odom->base_frame from VSLAM.'),
    ]
    
    # -----------------------------
    # NVBLOX lightweight preset
    # -----------------------------
    nvblox_light_params = {
        'num_cameras': 1,
        'mapping_type': 'static_tsdf',
        'input_qos': 'SENSOR_DATA',

        # frames
        'global_frame': odom_frame,
        'pose_frame': base_frame,
        'map_clearing_frame_id': base_frame,

        # memory/perf knobs
        'voxel_size': 0.10,
        'use_depth': True,
        'use_color': False,
        'use_lidar': False,

        # rate<=0 disables (mesh/layer/debug OFF)
        'tick_period_ms': 10,
        'integrate_depth_rate_hz': 10.0,
        'integrate_color_rate_hz': 0.0,
        'update_mesh_rate_hz': 0.0,
        'publish_layer_rate_hz': 0.0,
        'publish_debug_vis_rate_hz': 0.0,

        # minimal ESDF
        'esdf_mode': '2d',
        'publish_esdf_distance_slice': True,
        'update_esdf_rate_hz': 2.0,

        # queue / map growth limiting
        'maximum_input_queue_length': 3,
        'map_clearing_radius_m': 3.0,
        'clear_map_outside_radius_rate_hz': 2.0,

        # integration range
        'static_mapper.projective_integrator_max_integration_distance_m': 4.0,
        'static_mapper.projective_integrator_max_weight': 3.0,
        'static_mapper.raycast_subsampling_factor': 8,
        'static_mapper.esdf_integrator_max_distance_m': 1.5,
    }
        # odom -> base_link (임시; 나중에 vslam TF 정상 나오면 제거)
    static_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','odom','base_link'],
        output='screen'
    )
    
        # base_link -> camera_link (실측 extrinsic 넣어서 영구 유지)
    static_base_li_to_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','base_link','camera_link'],
        output='screen'
    )


    # 1) RealSense splitter
    splitter = ComposableNode(
        package='realsense_splitter',
        plugin='nvblox::RealsenseSplitterNode',
        name='realsense_splitter_node',
        namespace=camera_ns,
        parameters=[{
            'input_qos': 'SENSOR_DATA',
            'output_qos': 'SENSOR_DATA',
            'enable_image_denoising': False,
            'rectified_images': True,
            # NOTE: The rest of params are tolerated by this node in your environment.
            # If you see parameter warnings/errors, trim to the known splitter params above.
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'image_jitter_threshold_ms': 22.00,
        }],
        remappings=[
            ('input/infra_1',             ns_topic('infra1/image_rect_raw')),
            ('input/infra_1_metadata',    ns_topic('infra1/metadata')),
            ('input/infra_2',             ns_topic('infra2/image_rect_raw')),
            ('input/infra_2_metadata',    ns_topic('infra2/metadata')),
            ('input/depth',               ns_topic('depth/image_rect_raw')),
            ('input/depth_metadata',      ns_topic('depth/metadata')),
            ('input/pointcloud',          ns_topic('depth/color/points')),
            ('input/pointcloud_metadata', ns_topic('depth/metadata')),
        ],
    )

    # 2) Visual SLAM (stereo infra1+infra2 + IMU)
    vslam = ComposableNode(
        package='isaac_ros_visual_slam',
        plugin=vslam_plugin,
        name='visual_slam',
        namespace='visual_slam',
        parameters=[{
            # stereo
            'num_cameras': 2,
            'min_num_images': 2,

            # frames
            'map_frame': map_frame,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'imu_frame': imu_frame,

            # TF publication
            'publish_map_to_odom_tf': ParameterValue(publish_map_to_odom_tf, value_type=bool),
            'publish_odom_to_base_tf': ParameterValue(publish_odom_to_base_tf, value_type=bool),

            # main options
            'enable_imu_fusion': ParameterValue(enable_imu_fusion, value_type=bool),

            # This key may be ignored if the underlying VSLAM version doesn't support it.
            'enable_ground_constraint_in_odometry': ParameterValue(
                enable_ground_constraint_in_odometry, value_type=bool
            ),

            # Convenience
            'enable_rectified_pose': True,
            'rectified_images': True,
            'enable_localization_n_mapping': False,

            # debug views (can be heavy)
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,

            # RealSense optical frames (update if your TF names differ)
            'camera_optical_frames': [
                'camera_infra1_optical_frame',
                'camera_infra2_optical_frame',
            ],
        }],
        remappings=[
            ('visual_slam/camera_info_0', ns_topic('infra1/camera_info')),
            ('visual_slam/camera_info_1', ns_topic('infra2/camera_info')),
            ('visual_slam/image_0',       ns_topic('realsense_splitter_node/output/infra_1')),
            ('visual_slam/image_1',       ns_topic('realsense_splitter_node/output/infra_2')),
            ('visual_slam/imu',           ns_topic('imu')),
        ],
    )

    # 3) nvblox
    nvblox = ComposableNode(
        package='nvblox_ros',
        plugin=nvblox_plugin,
        name='nvblox_node',
        namespace='nvblox',
        # IMPORTANT: list of dicts (NOT a set)
        parameters=[nvblox_light_params],
        remappings=[
            ('camera_0/depth/image',       ns_topic('realsense_splitter_node/output/depth')),
            ('camera_0/depth/camera_info', ns_topic('depth/camera_info')),
            ('camera_0/color/image',       ns_topic('color/image_raw')),
            ('camera_0/color/camera_info', ns_topic('color/camera_info')),
        ],
    )

    # 4) cortexai manager
    cortexai_mgr = ComposableNode(
        package='cortexai_bringup',
        plugin='cortexai_bringup::CortexAIManager',
        name='cortexai_manager',
        namespace='cortexai',
        parameters=[{'enable_health_log': True}],
    )

    # -----------------------------
    # Containers
    # -----------------------------
    one_container = ComposableNodeContainer(
        name='cortexai_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[splitter, vslam, nvblox, cortexai_mgr],
        condition=IfCondition(use_one_container),
    )

    splitter_container = ComposableNodeContainer(
        name='cortexai_splitter_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[splitter],
        condition=UnlessCondition(use_one_container),
    )

    vslam_container = ComposableNodeContainer(
        name='cortexai_vslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[vslam],
        condition=UnlessCondition(use_one_container),
    )

    nvblox_container = ComposableNodeContainer(
        name='cortexai_nvblox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[nvblox],
        condition=UnlessCondition(use_one_container),
    )

    mgr_container = ComposableNodeContainer(
        name='cortexai_mgr_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[cortexai_mgr],
        condition=UnlessCondition(use_one_container),
    )

    return LaunchDescription(
        declare_args + [
            one_container,
            splitter_container,
            vslam_container,
            nvblox_container,
            mgr_container,
        ]
    )