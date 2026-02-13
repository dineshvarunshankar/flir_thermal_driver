from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch file for a single FLIR thermal camera without sync or telemetry.
    Simplified version for basic camera operation, recording, and control.
    """
    
    # Declare launch arguments
    declare_camera_name_arg = DeclareLaunchArgument(
        "camera_name", 
        default_value="thermal", 
        description="Name of the thermal camera"
    )
    
    declare_flir_id_arg = DeclareLaunchArgument(
        "flir_id", 
        default_value="34582", 
        description="FLIR camera serial ID"
    )
    
    # Raw-only pipeline; operation.raw is forced true in code
    
    # Palette selection is unified via operation.agc.color_palette (0-9) for both modes
    
    declare_frame_rate_arg = DeclareLaunchArgument(
        "frame_rate", 
        default_value="30", 
        description="Camera frame rate"
    )


    # Also allow operation-prefixed frame rate override
    declare_op_frame_rate_arg = DeclareLaunchArgument(
        "operation.frame_rate",
        default_value="",
        description="Override operation.frame_rate explicitly (leave empty to use frame_rate or YAML)"
    )
    
    # Hardware AGC-related launch overrides removed (raw-only pipeline)

    # Removed operation-prefixed palette override to avoid confusion; use agc_color_palette only

    # Telemetry removed (raw-only pipeline)
    
    # Temperature polling enable/disable
    declare_op_temperature_enabled_arg = DeclareLaunchArgument(
        "operation.temperature.enabled",
        default_value="true",
        description="Enable (true) or disable (false) periodic temperature polling"
    )

    # Processing enhancement toggle (only used when raw=true)
    declare_op_processing_enhance_arg = DeclareLaunchArgument(
        "operation.processing.enhance",
        default_value="",
        description="Override operation.processing.enhance (true/false). Leave empty to use YAML."
    )
    
    # Optional: per-launch log level to avoid setting global DEBUG
    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging severity threshold for this launch (debug, info, warn, error, fatal)"
    )
    
    # Deployment profile removed (raw-only pipeline)
    
    # Legacy colormap launch argument removed; use agc_color_palette instead
    
    # Get launch configurations
    camera_name = LaunchConfiguration("camera_name")
    flir_id = LaunchConfiguration("flir_id")
    # raw override removed
    frame_rate = LaunchConfiguration("frame_rate")
    op_frame_rate = LaunchConfiguration("operation.frame_rate")
    
    # AGC parameter configurations - Expanded Parameter Set
    # Hardware AGC and telemetry overrides removed
    op_temperature_enabled = LaunchConfiguration("operation.temperature.enabled")
    op_processing_enhance = LaunchConfiguration("operation.processing.enhance")
    deployment_profile = LaunchConfiguration("deployment_profile")
    log_level = LaunchConfiguration("log_level")
    # No colormap name support; palette is selected by agc_color_palette
    
    # Path to camera info file
    pkg_dir = get_package_share_directory("flir_ros_sync")
    camera_info_path = "file://" + os.path.join(
        pkg_dir, 
        "data", 
        "camera_info", 
        "example", 
        "thermal_caminfo_flir_640.yaml"
    )
    
    # Load the operational parameters file
    camera_params_file = os.path.join(
        pkg_dir,
        'config',
        'flir_camera_params.yaml'
    )
    
    # Build nodes inside OpaqueFunction so we can conditionally apply overrides
    def setup_nodes(context):
        # Resolve launch configurations
        op_frame_rate_v = op_frame_rate.perform(context)
        op_temperature_enabled_v = op_temperature_enabled.perform(context)
        op_processing_enhance_v = op_processing_enhance.perform(context)

        # Assemble overrides only when provided
        agc_overrides = {}

        # Assemble additional overrides for operation namespace
        op_overrides = {}
        if op_frame_rate_v:
            op_overrides["operation.frame_rate"] = int(op_frame_rate_v)
        # Telemetry removed
        if op_temperature_enabled_v:
            op_overrides["operation.temperature.enabled"] = (str(op_temperature_enabled_v).lower() == "true")
        if op_processing_enhance_v:
            op_overrides["operation.processing.enhance"] = (str(op_processing_enhance_v).lower() == "true")

        flir_camera_node = ComposableNode(
            package="flir_ros_sync",
            plugin="flir_ros_sync::FlirRos",
            name="flir_thermal_camera",
            parameters=[
                {
                    "device_name": "/dev/flir_boson_video",
                    "serial_port": "/dev/flir_boson_serial",
                    "intrinsic_url": camera_info_path,
                    "camera_name": camera_name,
                    "timestamp_offset": 0.0
                },
                camera_params_file,
                {
                    "operation.frame_rate": ParameterValue(frame_rate, value_type=int)
                },
                op_overrides if op_overrides else {},
                agc_overrides if agc_overrides else {}
            ],
            remappings=[
                ("/set_camera_info", [camera_name, "/set_camera_info"]),
            ]
        )

        camera_container = ComposableNodeContainer(
            name="flir_thermal_camera_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[flir_camera_node],
            output="screen"
        )

        # Legacy external thermal processor node (optional)
        legacy_params = [
            camera_params_file,
            {
                "camera_name": camera_name,
                "operation.frame_rate": ParameterValue(frame_rate, value_type=int)
            },
            op_overrides if op_overrides else {},
            agc_overrides if agc_overrides else {}
        ]

        thermal_process_node = Node(
            package="flir_ros_sync",
            executable="thermal_processor",
            name="thermal_processor",
            parameters=legacy_params,
            output="screen",
            condition=IfCondition(LaunchConfiguration("launch_legacy_processor"))
        )

        return [camera_container, thermal_process_node]
    
    # Optional legacy external thermal processor (DISABLED by default)
    # When raw=true, the camera node performs in-node AGC and publishes /image_processed.
    # Enable this node only for A/B testing by setting launch_legacy_processor:=true.
    declare_legacy_proc_arg = DeclareLaunchArgument(
        "launch_legacy_processor",
        default_value="false",
        description="Launch legacy external thermal_processor node (for A/B testing)."
    )

    # Node creation handled in setup_nodes()
    
    # CUDA Thermal Processor Node (PERMANENTLY DISABLED)
    # Disabled due to AGC interference issues - CUDA processor startup causes
    # contrast drops in main thermal processing pipeline regardless of timing
    # cuda_thermal_processor_node = Node(
    #     package="flir_ros_sync",
    #     executable="cuda_thermal_processor",
    #     name="cuda_thermal_processor",
    #     output="screen",
    #     condition=IfCondition(LaunchConfiguration("raw"))  # Only launch in raw mode
    # )
    # 
    # delayed_cuda_processor = TimerAction(
    #     period=10.0,
    #     actions=[cuda_thermal_processor_node]
    # )
    
    return LaunchDescription([
        declare_camera_name_arg,
        declare_flir_id_arg,
        # raw arg removed (raw-only pipeline)
        declare_frame_rate_arg,
        # hardware AGC and telemetry arg removed
        declare_op_temperature_enabled_arg,
        declare_op_frame_rate_arg,
        declare_log_level_arg,
        # deployment_profile removed
        declare_op_processing_enhance_arg,
        declare_legacy_proc_arg,
        OpaqueFunction(function=setup_nodes),
        # delayed_cuda_processor,  # CUDA processor DISABLED due to AGC interference
    ])
