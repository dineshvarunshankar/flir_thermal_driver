from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    """
    Simplified FLIR thermal camera launch file.
    Uses flir_camera_params.yaml for most parameters by default.
    Exposes high-level arguments for flexibility.
    """
    
    # Rely on ROS 2 unique node names/namespaces and graph behavior to avoid conflicts

    # --------------------------
    # Essential launch arguments
    # --------------------------
    declare_camera_name_arg = DeclareLaunchArgument(
        "camera_name", default_value="thermal",
        description="Logical camera name (used in topics)"
    )

    declare_node_name_arg = DeclareLaunchArgument(
        "node_name", default_value="flir_thermal_camera",
        description="ROS node name for the camera component"
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="",
        description="Namespace to group the camera node under"
    )


    declare_device_name_arg = DeclareLaunchArgument(
        "device_name", default_value="",
        description="Override: video device path for the FLIR camera. Leave empty to use YAML or launch default (/dev/flir_boson_video)."
    )

    declare_serial_port_arg = DeclareLaunchArgument(
        "serial_port", default_value="",
        description="Override: serial device path for the FLIR camera. Leave empty to use YAML or launch default (/dev/flir_boson_serial)."
    )


    declare_log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info",
        description="Logging severity threshold (debug, info, warn, error, fatal)"
    )

    declare_cuda_device_arg = DeclareLaunchArgument(
        "cuda_device", default_value="0",
        description="CUDA device ID to use for GPU processing (0, 1, 2, etc.)"
    )

    declare_legacy_proc_arg = DeclareLaunchArgument(
        "launch_legacy_processor", default_value="false",
        description="Launch legacy external thermal_processor node (for A/B testing)."
    )

    # --------------------------
    # Config file argument
    # --------------------------
    pkg_dir = get_package_share_directory("flir_ros_sync")
    default_config_file = os.path.join(pkg_dir, "config", "flir_camera_params.yaml")

    declare_config_file_arg = DeclareLaunchArgument(
        "config_file", default_value=default_config_file,
        description="Path to the FLIR camera parameter config file"
    )

    # --------------------------
    # Get other package paths
    # --------------------------
    camera_info_path = "file://" + os.path.join(
        pkg_dir, "data", "camera_info", "example", "thermal_caminfo_flir_640.yaml"
    )

    # --------------------------
    # Node setup function
    # --------------------------
    def setup_nodes(context):
        namespace = LaunchConfiguration("namespace").perform(context)
        node_name = LaunchConfiguration("node_name").perform(context)
        camera_name = LaunchConfiguration("camera_name")
        # raw-only pipeline; these overrides are removed

        device_name_cfg = LaunchConfiguration("device_name")
        serial_port_cfg = LaunchConfiguration("serial_port")
        config_file = LaunchConfiguration("config_file")

        # Build parameter list with YAML as the base; add overrides only if provided or required by fallback
        params_base = {
            "intrinsic_url": camera_info_path,
            "camera_name": camera_name,
            "timestamp_offset": 0.0,
        }
        
        # Evaluate config file path to inspect YAML for presence of parameters
        cfg_path = config_file.perform(context)
        yaml_has = set()
        try:
            with open(cfg_path, 'r') as f:
                data = yaml.safe_load(f) or {}
            # Support node-scoped YAML (node_name: ros__parameters: {...})
            if isinstance(data, dict):
                if node_name in data and isinstance(data[node_name], dict):
                    rp = data[node_name].get('ros__parameters', {})
                else:
                    # Try generic top-level ros__parameters
                    rp = data.get('ros__parameters', {})
                if isinstance(rp, dict):
                    # Flatten only top-level keys present (we care about device_name/serial_port)
                    yaml_has = set(rp.keys())
        except Exception as e:
            print(f"WARNING: Could not read config YAML '{cfg_path}': {e}")
            yaml_has = set()

        # Determine device_name following precedence
        device_name_cli = device_name_cfg.perform(context).strip()
        if device_name_cli:
            params_base["device_name"] = device_name_cli
        elif "device_name" not in yaml_has:
            # Fallback to launch default when YAML lacks the parameter
            params_base["device_name"] = "/dev/flir_boson_video"

        # Determine serial_port following precedence
        serial_port_cli = serial_port_cfg.perform(context).strip()
        if serial_port_cli:
            params_base["serial_port"] = serial_port_cli
        elif "serial_port" not in yaml_has:
            params_base["serial_port"] = "/dev/flir_boson_serial"

        # Initialize params with base dict and YAML file (YAML last would override base, so keep base first)
        params = [params_base, config_file]

        # Only override operation.raw when explicitly set to "true" or "false"
        # raw-only pipeline (operation.raw is forced true in code)

        # Main camera component
        flir_camera_node = ComposableNode(
            package="flir_ros_sync",
            plugin="flir_ros_sync::FlirRos",
            name=node_name,
            namespace=namespace,
            parameters=params,
            remappings=[
                ("/set_camera_info", [camera_name, "/set_camera_info"]),
            ],
            extra_arguments=[{
                'use_intra_process_comms': True,
                'env': [('CUDA_VISIBLE_DEVICES', LaunchConfiguration('cuda_device'))]
            }]
        )

        camera_container = ComposableNodeContainer(
            name=f"{node_name}_container",
            namespace=namespace,
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[flir_camera_node],
            output="screen"
        )

        # Optional legacy thermal processor
        thermal_process_node = Node(
            package="flir_ros_sync",
            executable="thermal_processor",
            name="thermal_processor",
            namespace=namespace,
            parameters=[config_file, {"camera_name": camera_name}],
            output="screen",
            condition=IfCondition(LaunchConfiguration("launch_legacy_processor"))
        )

        return [camera_container, thermal_process_node]

    # --------------------------
    # Launch description
    # --------------------------
    return LaunchDescription([
        # Ensure CUDA/NVIDIA device visibility inside the ROS container process
        SetEnvironmentVariable(name='NVIDIA_VISIBLE_DEVICES', value='all'),
        SetEnvironmentVariable(name='NVIDIA_DRIVER_CAPABILITIES', value='compute,utility'),
        # Explicitly avoid any accidental runtime disable flags
        SetEnvironmentVariable(name='FLIR_DISABLE_CUDA', value='0'),
        SetEnvironmentVariable(name='FLIR_WITH_CUDA', value='1'),
        declare_camera_name_arg,
        declare_node_name_arg,
        declare_namespace_arg,
        # raw arg removed (raw-only pipeline)
        declare_device_name_arg,
        declare_serial_port_arg,
        declare_log_level_arg,
        # deployment_profile removed (raw-only pipeline)
        declare_legacy_proc_arg,
        declare_config_file_arg,
        declare_cuda_device_arg,
        OpaqueFunction(function=setup_nodes),
    ])
