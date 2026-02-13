# FLIR Boson ROS Driver

This package provides a ROS 2 driver for FLIR Boson thermal cameras with robust AGC (Automatic Gain Control), hardware colormap support, and deployment-optimized configurations for mobile robotics applications.

## Key Features

- **Dual Operating Modes**: Raw 16-bit thermal data or hardware AGC 8-bit output
- **Hardware Colormaps**: Native Boson SDK colormap rendering (10 professional colormaps)
- **Software Processing**: Custom FLIR-style AGC with information-based histogram equalization
- **Deployment Profiles**: Optimized AGC settings for Ground UGV and Air UAV applications
- **Robust Configuration**: Advanced retry logic for reliable hardware parameter setting
- **OpenMP Acceleration**: Parallel processing for enhanced thermal image processing performance
- **Single YAML Configuration**: All camera and processing parameters in `src/flir_ros_sync/config/flir_camera_params.yaml`
- **Real-time Performance**: 30 FPS operation with minimal computational overhead
- **Core Temperature Monitoring**: Real-time camera thermal state monitoring

## Installation

> **Note**: Steps 1-4 are not needed if running inside the airlab_ws basestation Docker container environment, as all dependencies are pre-installed.

### 1. Install ROS 2

Ensure ROS 2 is installed on your system. Follow the [official ROS 2 installation guide](https://docs.ros.org/en/) for instructions specific to your ROS 2 distribution.

### 2. Package Build (ROS 2)

1. Install the required ROS 2 packages:

   ```bash
   sudo apt install ros-humble-image-transport ros-humble-cv-bridge ros-humble-camera-info-manager
   ```

2. Install **flirpy** for interfacing with the FLIR Boson cameras via a simple Python API:

   ```bash
   pip install flirpy
   ```

3. Ensure OpenCV is installed.

4. Install V4L2 headers for Linux:

   ```bash
   sudo apt install libv4l-dev
   ```

5. If needed, recompile the FSLP libraries (for FLIR Boson SDK):
   - Navigate to the Boson SDK folder and then the `FSLP_Files` subfolder.
   - Run `make` or `make all`, which attempts to build a 64-bit library based on your platform.

6. Copy udev rules to create device symlinks for easier setup:

   ```bash
   sudo cp 42-flir-ros.rules /etc/udev/rules.d
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

7. Clone this package into your ROS workspace and build it:

   ```bash
   colcon build --packages-select flir_ros_sync
   ```

8. Source the ROS 2 workspace after building:

   ```bash
   source install/setup.bash
   ```

### 3. Review and Configure Launch Files

The package includes a simplified launch file for single camera operation:

- **triage.launch.py**: Located in `src/flir_ros_sync/launch/example/`
- Configured for single-camera operation with sensible defaults
- Telemetry is disabled by default and only used with raw mode
- The launch file automatically detects your camera device and configures parameters

## Usage (Single Camera)

1. **Verify Camera Connection**: Ensure your FLIR Boson camera is connected via USB and detected by the system:

   ```bash
   ls /dev/video*        # Check for video devices
   ls /dev/serial/by-id  # Check for serial devices
   ```

2. **Launch Options**:

   **Hardware AGC Mode (Recommended for most applications)**:
   ```bash
   # Basic hardware AGC with optimized defaults
   ros2 launch flir_ros_sync triage.launch.py raw:=false
   
   # Hardware AGC with specific palette (IDs 0-9, e.g., 0=WhiteHot, 1=BlackHot)
   ros2 launch flir_ros_sync triage.launch.py raw:=false agc_color_palette:=0
   
   # Night, ground: person + obstacle awareness (WhiteHot)
   ros2 launch flir_ros_sync triage.launch.py raw:=false \
     agc_color_palette:=0 agc_gamma:=0.70 agc_max_gain:=2.5 agc_linear_percent:=45.0 \
     agc_plateau:=10.0 agc_tail_rejection:=3.0 agc_dde:=1.20 agc_detail_headroom:=14.0 agc_smoothing_factor:=1200.0 \
     agc_detail_headroom:=14.0 agc_damping_factor:=25.0
   
   # High detail imaging with custom AGC settings
   ros2 launch flir_ros_sync triage.launch.py raw:=false \
     agc_gamma:=0.6 agc_max_gain:=1.8 agc_dde:=1.5 agc_smoothing_factor:=1500 agc_color_palette:=2
   ```

   **Raw Mode (For custom processing and software colormaps)**:
   ```bash
   # Raw 16-bit thermal data with software palette (IDs 0-9, e.g., 0=WhiteHot)
   ros2 launch flir_ros_sync triage.launch.py raw:=true agc_color_palette:=0
   
   # Raw mode with AGC damping for flicker reduction
   ros2 launch flir_ros_sync triage.launch.py raw:=true agc_damping_factor:=80
   ```

3. **Launch Parameters**:

   **Basic Parameters**:
   - `camera_name`: Camera namespace (default: "thermal")
   - `flir_id`: Camera serial ID (default: "34582")
   - `frame_rate`: Camera frame rate in Hz (default: 30)

   **Operating Mode**:
   - `raw`: Enable raw mode (default: false)
     - `true`: 16-bit raw thermal data with software processing
     - `false`: 8-bit hardware AGC processed data

   **Palette Selection (Unified)**:
   - Launch override: `agc_color_palette` (int, 0–9)
   - YAML source of truth: `operation.agc.color_palette` (int, 0–9)
   - IDs: 0=WhiteHot, 1=BlackHot, 2=Rainbow, 3=Rainbow_HC, 4=Ironbow, 5=Lava, 6=Arctic, 7=Globow, 8=Graded_Fire, 9=Hottest
   
   **Deployment Profiles** (for `raw=false` hardware AGC mode):
   - `deployment_profile`: Optimized AGC settings (default: "ground")
     - `ground`: Ground UGV profile - optimized for mobile robots
     - `air`: Air UAV profile - optimized for aerial drones
     - `custom`: Use individual AGC parameters

   **Configuration Files**:
   - Camera parameters: `src/flir_ros_sync/config/flir_camera_params.yaml`

## Nighttime Outdoor People Scenarios – Launch Recipes

The following recipes target outdoor, night-time scenes for Ground UGV, with Air UAV variants for (i) and (ii). Palette IDs are unified (0–9). Use either RViz2 or RQt to visualize:

```bash
# RViz2
rviz2

# RQt (recommended for quick checks)
ros2 run rqt_image_view rqt_image_view /thermal/image_processed
```

### (i) People Detection (small targets, edge emphasis, low noise)

- Recommended palette: BlackHot (ID 1)
- Behavior: lower max_gain, stronger edges, moderate damping, suppress hot/cold outliers

Hardware AGC (raw=false):
```bash
ros2 launch flir_ros_sync single_thermal_camera.launch.py raw:=false \
  deployment_profile:=ground agc_color_palette:=1 \
  agc_gamma:=0.75 agc_max_gain:=1.8 agc_linear_percent:=30.0 \
  agc_tail_rejection:=2.0 agc_dde:=1.5 agc_detail_headroom:=18.0 \
  agc_smoothing_factor:=1400.0 agc_damping_factor:=20.0
```

Air UAV variant (faster motion stability):
```bash
ros2 launch flir_ros_sync single_thermal_camera.launch.py raw:=false \
  deployment_profile:=air agc_color_palette:=1 \
  agc_gamma:=0.75 agc_max_gain:=1.6 agc_linear_percent:=35.0 \
  agc_tail_rejection:=2.5 agc_dde:=1.5 agc_detail_headroom:=18.0 \
  agc_smoothing_factor:=1500.0 agc_damping_factor:=30.0
```

Software mode (raw=true):
```bash
ros2 launch flir_ros_sync single_thermal_camera.launch.py raw:=true \
  agc_color_palette:=1 agc_gamma:=0.75 agc_max_gain:=1.8 agc_linear_percent:=30.0 \
  agc_tail_rejection:=2.0 agc_dde:=1.5 agc_detail_headroom:=18.0 \
  agc_smoothing_factor:=1400.0 agc_damping_factor:=20.0
```

### (ii) General Person Inspection (balanced detail in center, surroundings visible)

- Recommended palette: WhiteHot (ID 0)
- Behavior: balanced gamma and gain, moderate damping; approximate center emphasis by tuning linear_percent/plateau

Hardware AGC (raw=false):
```bash
ros2 launch flir_ros_sync single_thermal_camera.launch.py raw:=false \
  deployment_profile:=ground agc_color_palette:=0 \
  agc_gamma:=0.80 agc_max_gain:=2.0 agc_linear_percent:=35.0 \
  agc_plateau:=8.0 agc_tail_rejection:=1.5 agc_dde:=1.3 \
  agc_detail_headroom:=18.0 agc_smoothing_factor:=1300.0 agc_damping_factor:=15.0
```

Air UAV variant:
```bash
ros2 launch flir_ros_sync single_thermal_camera.launch.py raw:=false \
  deployment_profile:=air agc_color_palette:=0 \
  agc_gamma:=0.80 agc_max_gain:=1.9 agc_linear_percent:=35.0 \
  agc_plateau:=9.0 agc_tail_rejection:=2.0 agc_dde:=1.3 \
  agc_detail_headroom:=18.0 agc_smoothing_factor:=1450.0 agc_damping_factor:=25.0
```

Software mode (raw=true):
```bash
ros2 launch flir_ros_sync single_thermal_camera.launch.py raw:=true \
  agc_color_palette:=0 agc_gamma:=0.80 agc_max_gain:=2.0 agc_linear_percent:=35.0 \
  agc_plateau:=8.0 agc_tail_rejection:=1.5 agc_dde:=1.3 \
  agc_detail_headroom:=18.0 agc_smoothing_factor:=1300.0 agc_damping_factor:=15.0
```

### (iii) Close-up Person Inspection (minute motion like blinking, pulse)

- Recommended palette: WhiteHot (ID 0). Optional: Ironbow (ID 4) for subtle gradients
- Behavior: slightly lower gamma for pop, more smoothing, allow slightly higher gain, strong local detail

Hardware AGC (raw=false):
```bash
ros2 launch flir_ros_sync single_thermal_camera.launch.py raw:=false \
  deployment_profile:=ground agc_color_palette:=0 \
  agc_gamma:=0.70 agc_max_gain:=2.2 agc_linear_percent:=30.0 \
  agc_plateau:=7.0 agc_tail_rejection:=1.0 agc_dde:=1.4 \
  agc_detail_headroom:=20.0 agc_smoothing_factor:=1600.0 agc_damping_factor:=10.0
```

Optional alternate palette:
```bash
ros2 launch flir_ros_sync single_thermal_camera.launch.py raw:=false \
  deployment_profile:=ground agc_color_palette:=4 \
  agc_gamma:=0.70 agc_max_gain:=2.2 agc_linear_percent:=30.0 \
  agc_plateau:=7.0 agc_tail_rejection:=1.0 agc_dde:=1.4 \
  agc_detail_headroom:=20.0 agc_smoothing_factor:=1600.0 agc_damping_factor:=10.0
```

Software mode (raw=true):
```bash
ros2 launch flir_ros_sync single_thermal_camera.launch.py raw:=true \
  agc_color_palette:=0 agc_gamma:=0.70 agc_max_gain:=2.2 agc_linear_percent:=30.0 \
  agc_plateau:=7.0 agc_tail_rejection:=1.0 agc_dde:=1.4 \
  agc_detail_headroom:=20.0 agc_smoothing_factor:=1600.0 agc_damping_factor:=10.0
```

Notes:
- Center-ROI AGC is not currently exposed; the above settings approximate center emphasis via `agc_linear_percent` and `agc_plateau` without new parameters.
- Keep palette IDs consistent: 10 palettes (0–9) supported in both modes via `agc_color_palette`.

## Thermal Colormaps

The system supports **dual operating modes** with different visualization capabilities:

- **Hardware AGC Mode (`raw:=false`)**: Native Boson SDK colormap rendering controlled by palette ID (0–9)
- **Software Processing Mode (`raw:=true`)**: Information-based processing with FLIR-style palettes mapped to the same IDs (0–9)

> **📋 For detailed colormap information, usage examples, and application-specific recommendations, see [THERMAL_COLORMAPS.md](THERMAL_COLORMAPS.md)**

## Published Topics

The driver publishes different topics depending on the selected mode:

### Raw Mode (`raw=true`)
- `/thermal/image` - Raw 16-bit thermal data (sensor_msgs/Image)
- `/thermal/image_processed` - Enhanced 8-bit processed thermal image (sensor_msgs/Image)
- `/thermal/camera_info` - Camera calibration information (sensor_msgs/CameraInfo)
- `/thermal/core_temperature` - Camera core temperature in Celsius (std_msgs::msg::Float32)

### Hardware AGC Mode (`raw=false`)
- `/thermal/image` - 8-bit hardware AGC processed thermal image (sensor_msgs/Image)
- `/thermal/camera_info` - Camera calibration information (sensor_msgs/CameraInfo)
- `/thermal/core_temperature` - Camera core temperature in Celsius (std_msgs/Float32)

## Configuration

### Parameter Configuration (YAML)

All camera and processing parameters are configured via YAML for maintainability and flexibility. The configuration file is located at:

```
src/flir_ros_sync/config/flir_camera_params.yaml
```

#### Structure of the YAML Configuration

The YAML file contains several sections:

1. **Camera Intrinsics** (required for calibration):
   - `image_width`, `image_height`: Camera resolution
   - `camera_matrix`: Intrinsic camera matrix (3x3)
   - `distortion_coefficients`: Lens distortion parameters
   - `rectification_matrix`: Stereo rectification matrix
   - `projection_matrix`: Camera projection matrix

2. **Operation Parameters** (can be modified as needed):
   ```yaml
   operation:
     # Basic camera settings
     raw: false         # Enable hardware AGC processing (false) or raw thermal data (true)
     frame_rate: 30     # Desired frame rate in Hz
     gain_mode: 0       # Gain mode (0=High, 1=Low, 2=Auto)
     ffc_mode: 1        # Flat Field Correction mode (0=Manual, 1=Auto, 2=External)
     
     # Telemetry settings
     telemetry:
       enabled: false   # Enable telemetry data (only used in raw mode; disabled by default)
       location: 1      # Telemetry location (0=off, 1=bottom, 2=top). Ignored unless raw mode with telemetry enabled
     
     # AGC (Automatic Gain Control) settings - 11 parameters
     agc:
       gamma: 0.8         # Adaptive Contrast Enhancement (ACE): recommended <1 for more feature pop (0.5-4.0 valid)
       max_gain: 2.0      # Max Gain limits max contrast scaling; 2.0 recommended for detail preservation (0.25-8.0 valid)
       linear_percent: 30 # Linear percent (0-100), 30% helps both hot/cold feature mapping and subtle anomaly preservation
       plateau: 7         # Plateau (1-100), sets the percent of pixels allowed in a histogram bin; 7 is default; higher increases local contrast
       tail_rejection: 2  # Tail rejection (0-50), ignores top/bottom % histogram to reject hot/cold outliers; higher = more outlier removal
       outlier_cut_balance: 1 # Outlier cut balance (0=ignore low end, 1=balanced [default], 2=ignore high end)
       dde: 1.25          # Digital Detail Enhancement (0.8-2.0), sharpens edges. 1.0 default, 1.25 for more detail
       detail_headroom: 16.0 # Detail Headroom (0-255), typical values are 8-24. Promotes local edge detail when DDE enabled
       smoothing_factor: 1250.0 # Smoothing Factor (recommended default: 1250), higher values preserve finer edges when using DDE
       damping_factor: 5  # Damping factor for AGC adaptation (0-100, 0=fast AGC, 100=almost frozen); 5 is fast but smooth
       color_palette: 0   # Unified palette ID (0=WhiteHot, 1=BlackHot, 2=Rainbow, 3=Rainbow_HC, 4=Ironbow, 5=Lava, 6=Arctic, 7=Globow, 8=Graded_Fire, 9=Hottest)
     
     # Image processing settings (for raw mode only)
     processing:
       rectify: false   # Enable image rectification
       enhance: true    # Enable image enhancement
       method: "information_based"  # Processing method (e.g., "information_based")
       # Use operation.agc.color_palette (0-9) as the single source of truth in both modes
     # Other settings
     # (No top-level 'colormap' parameter; use operation.agc.color_palette only)
   ```

3. **Advanced Settings**:
   - `ffc`: Flat Field Correction settings
   - `temperature`: Temperature monitoring controls
     - `operation.temperature.enabled`: Enable/disable periodic temperature polling (default: false)
     - `operation.temperature.warning_threshold`: Warn if core temp exceeds this value (°C)
     - `operation.temperature.critical_threshold`: Critical threshold (°C)

   Note: External sync has been removed for single-camera setups to simplify the driver.

#### Validating Configuration

Configuration is validated at node startup. The driver logs all resolved key parameters (device paths, frame rate, AGC parameters, telemetry settings). Review the node logs after launch to confirm values from `src/flir_ros_sync/config/flir_camera_params.yaml` were applied. No external Python validation script is required.

## AGC Parameters and Presets

For the full list of AGC parameters, recommended ranges, and example presets, see THERMAL_COLORMAPS.md. That file contains up-to-date launch/YAML examples and application-specific recommendations. README keeps only the essentials and links out for details.

### When to Use Raw vs Hardware AGC Mode

**Raw Mode (`raw: true`)**:
- Use for: custom processing and software visualization
- Output: 16-bit raw thermal data + 8-bit processed visualization
- Processing: Software-based in ROS 2 node
- AGC Parameters: Used by software pipeline (gamma, damping_factor, etc.)

**AGC Mode (`raw: false`) - Recommended**:
- Use for: Real-time visualization, person/obstacle detection, edge computing
- Output: 8-bit processed thermal image (mono8/BGR8)
- Processing: Hardware-based in FLIR Boson camera
- AGC Parameters: **Applied with 11-parameter control**



## Hardware Configuration Reliability

The driver implements **robust retry logic** for reliable hardware parameter setting with 5-attempt retry mechanism and comprehensive error handling for SDK communication issues.

## Monitoring Camera Status

**Telemetry**: Disabled by default. When enabled, telemetry rows are only used in raw mode.

To view thermal images (requires image visualization tools):
```bash
ros2 run rqt_image_view rqt_image_view /thermal/image_processed
```

## Notes

* The driver is optimized for single camera operation; telemetry is disabled by default and only applicable in raw mode
* Raw thermal data is published as 16-bit images (640x512 pixels) for maximum temperature resolution
* The camera performs automatic FFC (Flat Field Correction) in mode 1 - no manual intervention required
* When enabled in raw mode, telemetry provides additional data lines (+2 rows) with timestamp and camera status
* Camera calibration is provided via the `CameraInfo` topic (`/thermal/camera_info`); load your calibration via YAML in your workspace
* All buffer overflow issues and unsafe string handling have been resolved for stable operation