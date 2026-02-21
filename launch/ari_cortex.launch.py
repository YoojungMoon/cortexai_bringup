#============Aero Robotics Innovtions=============
# cortexai_perception.launch.py
# - add: nvblox lightweight preset to avoid CUDA OOM
# 02.17.2026
# By BG Kang
#
# Fixes in this revision:
# - Ensure LaunchArguments (vslam_plugin / nvblox_plugin / use_one_container / camera_ns) are declared in LaunchDescription
# - Use camera_ns consistently (no hard-coded '/camera/..' remaps)
# - Optional: support one-container vs multi-container mode via use_one_container
#
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    camera_ns = LaunchConfiguration('camera_ns')
    use_one_container = LaunchConfiguration('use_one_container')

    # Isaac ROS 릴리즈마다 plugin 문자열이 다를 수 있어 "한 군데에서만" 바꿀 수 있게 args로 둠
    vslam_plugin = LaunchConfiguration('vslam_plugin')
    nvblox_plugin = LaunchConfiguration('nvblox_plugin')

    # topic helper (LaunchConfiguration 안전하게 붙이기)
    def ns_topic(tail: str):
        # 결과: "/<camera_ns>/<tail>"
        return ['/', camera_ns, '/', tail.lstrip('/')]

    declare_args = [
        DeclareLaunchArgument('camera_ns', default_value='camera'),

        # 성능 최적: 하나의 컨테이너에 4개를 모두 넣는 옵션
        DeclareLaunchArgument('use_one_container', default_value='true'),

        # plugin names (release-dependent)
        DeclareLaunchArgument(
            'vslam_plugin',
            default_value='nvidia::isaac_ros::visual_slam::VisualSlamNode'
        ),
        DeclareLaunchArgument(
            'nvblox_plugin',
            default_value='nvblox::NvbloxNode'
        ),
    ]

    # -----------------------------
    # NVBLOX lightweight preset
    # -----------------------------
    nvblox_light_params = {
        'num_cameras': 1,
        'mapping_type': 'static_tsdf',
        'input_qos': 'SENSOR_DATA',

        # 메모리 절감 핵심
        'voxel_size': 0.10,
        'use_depth': True,
        'use_color': False,
        'use_lidar': False,

        # rate<=0이면 비활성 (mesh/layer/debug OFF)
        'tick_period_ms': 10,
        'integrate_depth_rate_hz': 10.0,
        'integrate_color_rate_hz': 0.0,
        'update_mesh_rate_hz': 0.0,
        'publish_layer_rate_hz': 0.0,
        'publish_debug_vis_rate_hz': 0.0,

        # ESDF 최소만
        'esdf_mode': '2d',
        'publish_esdf_distance_slice': True,
        'update_esdf_rate_hz': 2.0,

        # 큐/맵 성장 제한
        'maximum_input_queue_length': 3,
        'map_clearing_radius_m': 3.0,
        'clear_map_outside_radius_rate_hz': 2.0,
        'map_clearing_frame_id': 'base_link',

        # integration 범위 축소
        'static_mapper.projective_integrator_max_integration_distance_m': 4.0,
        'static_mapper.projective_integrator_max_weight': 3.0,
        'static_mapper.raycast_subsampling_factor': 8,
        'static_mapper.esdf_integrator_max_distance_m': 1.5,
    }

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
            'enable_imu_fusion': True,
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

    # 2) Visual SLAM
    vslam = ComposableNode(
        package='isaac_ros_visual_slam',
        plugin=vslam_plugin,
        name='visual_slam',
        namespace='visual_slam',
        parameters=[{
            
            'base_frame': 'camera_link',
            'imu_frame': 'camera_gyro_optical_frame',
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'camera_optical_frames': [
                'camera_infra1_optical_frame',
                'camera_infra2_optical_frame',
            ]
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
        parameters=[{nvblox_light_params}],
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

    # 1-container mode
    one_container = ComposableNodeContainer(
        name='cortexai_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[splitter, vslam, nvblox, cortexai_mgr],
        condition=IfCondition(use_one_container),
    )

    # multi-container mode (debug/isolation)
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