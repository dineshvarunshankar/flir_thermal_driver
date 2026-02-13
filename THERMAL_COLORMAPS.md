# FLIR Thermal Camera Colormaps Guide

## Overview
The FLIR thermal camera system supports **dual operating modes** with both hardware-accelerated AGC processing and software-based thermal processing. The system provides **10 hardware colormaps** via native Boson SDK rendering and comprehensive software AGC processing with information-based histogram equalization for enhanced thermal image interpretation across different applications and environments.

## Operating Modes

### Hardware AGC Mode (`raw:=false` - **Recommended**)
- **Native Boson SDK rendering** with hardware acceleration
- **10 professional colormaps** including FLIR industry standards
- **Better performance** and authentic FLIR camera behavior
- **Robust retry logic** for reliable hardware configuration
- **Works with deployment profiles** (Ground UGV, Air UAV)
- **Enterprise-grade reliability** with comprehensive error handling
- **Real-time core temperature monitoring**

### Software Processing Mode (`raw:=true`)
- **Information-based histogram equalization** with OpenMP acceleration
- **Custom FLIR-style AGC processing** optimized for thermal applications
- **Full raw 16-bit data access** for custom processing pipelines
- **Single YAML configuration** via `src/flir_ros_sync/config/flir_camera_params.yaml`
- **Enhanced processing methods** (information_based, plateau)

## Hardware Colormaps (`raw:=false` - **Recommended**)

**Native Boson SDK rendering with hardware acceleration and robust retry logic:**

**Configuration**: Use `operation.agc.color_palette` (0–9) in YAML; use `agc_color_palette:=<id>` in launch.

### 🔥 **Grayscale Palettes**

#### 1. **White Hot** (Default)
- **Usage**: `raw:=false agc_color_palette:=0`
- **SDK ID**: 0 (FLR_COLORLUT_WHITEHOT)
- **Description**: Default grayscale, hot = white
- **Best for**: General-purpose applications, mixed terrain, urban environments
- **Advantages**: Versatile for wide temperature spans, authentic FLIR rendering

#### 2. **Black Hot**
- **Usage**: `raw:=false agc_color_palette:=1`
- **SDK ID**: 1 (FLR_COLORLUT_BLACKHOT)
- **Description**: Inverted grayscale, hot = black
- **Best for**: Search & rescue, fire operations, law enforcement, nighttime wildlife surveys
- **Advantages**: Body heat appears as dark shapes, reduces screen glare, professional FLIR behavior

### 🌈 **High-Contrast Color Palettes**

#### 3. **Rainbow**
- **Usage**: `raw:=false agc_color_palette:=2`
- **SDK ID**: 2 (FLR_COLORLUT_RAINBOW)
- **Description**: Full spectrum rainbow
- **Best for**: Electrical inspections, scientific research, educational settings
- **Advantages**: Excellent for detecting subtle temperature differences

#### 4. **Rainbow High Contrast**
- **Usage**: `raw:=false agc_color_palette:=3`
- **SDK ID**: 3 (FLR_COLORLUT_RAINBOW_HC)
- **Description**: High contrast rainbow variant
- **Best for**: Low-contrast scenes, building surveys, minimal heat change environments
- **Advantages**: Amplifies subtle temperature variations with hardware acceleration

#### 5. **Ironbow**
- **Usage**: `raw:=false agc_color_palette:=4`
- **SDK ID**: 4 (FLR_COLORLUT_IRONBOW)
- **Description**: Iron/metal color scheme
- **Best for**: Solar panel inspections, industrial maintenance, electrical inspections
- **Advantages**: Quick identification of broken components, native FLIR iron palette

#### 6. **Lava**
- **Usage**: `raw:=false agc_color_palette:=5`
- **SDK ID**: 5 (FLR_COLORLUT_LAVA)
- **Description**: Red/orange/yellow lava colors
- **Best for**: Roof inspections, insulation analysis, solar panel assessments
- **Advantages**: Quick heat pattern identification, building envelope analysis

#### 7. **Arctic**
- **Usage**: `raw:=false agc_color_palette:=6`
- **SDK ID**: 6 (FLR_COLORLUT_ARCTIC)
- **Description**: Blue/white arctic colors
- **Best for**: HVAC inspections, cool background detection, ice analysis
- **Advantages**: Excellent for cold temperature visualization, professional arctic palette

#### 8. **Globow**
- **Usage**: `raw:=false agc_color_palette:=7`
- **SDK ID**: 7 (FLR_COLORLUT_GLOBOW)
- **Description**: Global rainbow variant
- **Best for**: Scientific analysis, research applications, global temperature mapping
- **Advantages**: Enhanced global temperature visualization with FLIR optimization

#### 9. **Graded Fire**
- **Usage**: `raw:=false agc_color_palette:=8`
- **SDK ID**: 8 (FLR_COLORLUT_GRADED_FIRE)
- **Description**: Fire-based color gradient
- **Best for**: Fire detection, combustion analysis, high-temperature monitoring
- **Advantages**: Optimized for fire and extreme heat visualization

#### 10. **Hottest**
- **Usage**: `raw:=false agc_color_palette:=9`
- **SDK ID**: 9 (FLR_COLORLUT_HOTTEST)
- **Description**: Hottest temperature highlighting
- **Best for**: Peak temperature detection, hotspot identification, safety monitoring
- **Advantages**: Emphasizes highest temperatures for critical monitoring applications

## Software Processing Mode (`raw:=true`)

Information-based histogram equalization with OpenMP acceleration and FLIR-style palettes (IDs 0–9).

Configuration: Set via `operation.agc.color_palette` (0–9) in YAML or `agc_color_palette:=<id>` in launch.

### Processing Features
- **Information-based histogram equalization** (default method)
- **Plateau-based processing** (alternative method)
- **10 unified palettes (IDs 0–9)** matching hardware mode, with OpenCV/custom implementations
- **OpenMP parallel processing** for enhanced performance
- **Modular AGC parameters** matching hardware mode capabilities
- **Real-time processing** at 30 FPS with minimal overhead

### Software Colormaps

#### 🔥 **Grayscale Palettes** (IDs)

##### 1. **White Hot** (Default)
- **Usage**: `operation.agc.color_palette: 0` in config or `agc_color_palette:=0` via launch
- **Implementation**: Grayscale conversion (cv::COLOR_GRAY2BGR)
- **Description**: Hot objects appear white, cold objects appear black
- **Best for**: General-purpose applications, mixed terrain, urban environments
- **Advantages**: Versatile for wide temperature spans, consistent contrast

##### 2. **Black Hot**
- **Usage**: `operation.agc.color_palette: 1` in config or `agc_color_palette:=1` via launch
- **Implementation**: Inverted grayscale (255 - image)
- **Description**: Hot objects appear black, cold objects appear white
- **Best for**: Search & rescue, fire operations, law enforcement, nighttime wildlife surveys
- **Advantages**: Body heat appears as dark shapes, reduces screen glare

#### 🌈 **High-Contrast Color Palettes**

##### 4. **Rainbow**
- **Usage**: `operation.agc.color_palette: 2` in config or `agc_color_palette:=2` via launch
- **Implementation**: OpenCV COLORMAP_RAINBOW (proxy)
- **Description**: Full spectrum colors for precise temperature differentiation
- **Best for**: Electrical inspections, scientific research, educational settings
- **Advantages**: Excellent for detecting subtle temperature differences

##### 5. **Rainbow High Contrast**
- **Usage**: `operation.agc.color_palette: 3` in config or `agc_color_palette:=3` via launch
- **Implementation**: OpenCV proxy with added contrast shaping
- **Description**: Enhanced color separation with histogram equalization
- **Best for**: Low-contrast scenes, building surveys, minimal heat change environments
- **Advantages**: Amplifies subtle temperature variations

##### 6. **Ironbow**
- **Usage**: `operation.agc.color_palette: 4` in config or `agc_color_palette:=4` via launch
- **Implementation**: Custom LUT / OpenCV proxy
- **Description**: Iron-red color scheme for anomaly detection
- **Best for**: Solar panel inspections, industrial maintenance, electrical inspections
- **Advantages**: Quick identification of broken components, heat distribution analysis

##### 7. **Lava**
- **Usage**: `operation.agc.color_palette: 5` in config or `agc_color_palette:=5` via launch
- **Implementation**: Custom gradient (Black→Red→Orange→Yellow→White)
- **Description**: Bold red-orange contrast with custom gradient
- **Best for**: Roof inspections, insulation analysis, solar panel assessments
- **Advantages**: Quick heat pattern identification, building envelope analysis

##### 8. **Arctic**
- **Usage**: `operation.agc.color_palette: 6` in config or `agc_color_palette:=6` via launch
- **Implementation**: OpenCV COLORMAP_WINTER
- **Description**: Cool blue-white scheme
- **Best for**: HVAC system inspections, detecting warm objects against cool backgrounds
- **Advantages**: Excellent for climate control system analysis, leak detection

### AGC Processing Methods

#### 1. **Information-Based** (Default)
- **Usage**: `method: "information_based"` in config
- **Description**: Advanced histogram equalization optimized for thermal data
- **Best for**: General-purpose applications, detection tasks, mobile robotics
- **Advantages**: Optimal contrast enhancement, preserves thermal details, OpenMP accelerated

#### 2. **Plateau-Based**
- **Usage**: `method: "plateau"` in config
- **Description**: Plateau-based histogram processing for specialized applications
- **Best for**: Specific thermal analysis requirements
- **Advantages**: Alternative processing approach for specialized use cases

### Configuration Examples

**Basic Configuration File:**
```yaml
# flir_camera_params.yaml
flir_thermal_camera:
  ros__parameters:
    operation:
      processing:
        method: "information_based"  # or "plateau"
        rectify: false
        enhance: true
      agc:
        gamma: 0.8
        max_gain: 2.0
        linear_percent: 30.0
        # ... all other AGC parameters
        color_palette: 0  # 0=WhiteHot, 1=BlackHot, 2=Rainbow, 3=Rainbow HC, 4=Ironbow, 5=Lava, 6=Arctic, 7=Globow, 8=Graded Fire, 9=Hottest
```

### Usage Examples

### Hardware AGC Mode (Recommended)

**Basic Hardware AGC Usage:**
```bash
# Default Ground UGV profile with white_hot
ros2 launch flir_ros_sync triage.launch.py raw:=false

# Black Hot for Search & Rescue (ID 1)
ros2 launch flir_ros_sync triage.launch.py raw:=false agc_color_palette:=1

```

**Hardware AGC with Deployment Profiles:**
```bash
# Air UAV profile with Rainbow (ID 2)
ros2 launch flir_ros_sync triage.launch.py raw:=false deployment_profile:=air agc_color_palette:=2

# Ground UGV profile with Black Hot (ID 1) for detection
ros2 launch flir_ros_sync triage.launch.py raw:=false deployment_profile:=ground agc_color_palette:=1

```

### Software Processing Mode

**Software Colormap Usage:**
```bash
# Default white_hot with information-based processing
ros2 launch flir_ros_sync triage.launch.py raw:=true

# Black Hot (ID 1) for search & rescue
ros2 launch flir_ros_sync triage.launch.py raw:=true agc_color_palette:=1

```

**Processing Method Configuration:**
```bash
# Software processing (method, rectification, and other options are configured in flir_camera_params.yaml)
ros2 launch flir_ros_sync triage.launch.py raw:=true
```
Note: Configure `operation.processing.method`, `rectify`, `enhance`, and AGC parameters in `flir_camera_params.yaml`. Use `agc_color_palette:=<id>` on the command line to override `operation.agc.color_palette`.

### Nighttime Outdoor People Scenarios – Launch Recipes

Use either RViz2 or RQt to visualize results:

```bash
# RViz2
rviz2

# RQt (quick view)
ros2 run rqt_image_view rqt_image_view /thermal/image_processed
```

#### (i) People Detection (small targets, edge emphasis, low noise)

- Palette: BlackHot (ID 1)
- Lower max_gain, stronger edges, moderate damping, outlier suppression

Hardware AGC (raw=false, Ground UGV):
```bash
ros2 launch flir_ros_sync triage.launch.py raw:=false \
  deployment_profile:=ground agc_color_palette:=1 \
  agc_gamma:=0.75 agc_max_gain:=1.8 agc_linear_percent:=30.0 \
  agc_tail_rejection:=2.0 agc_dde:=1.5 agc_detail_headroom:=18.0 \
  agc_smoothing_factor:=1400.0 agc_damping_factor:=20.0
```

Air UAV variant:
```bash
ros2 launch flir_ros_sync triage.launch.py raw:=false \
  deployment_profile:=air agc_color_palette:=1 \
  agc_gamma:=0.75 agc_max_gain:=1.6 agc_linear_percent:=35.0 \
  agc_tail_rejection:=2.5 agc_dde:=1.5 agc_detail_headroom:=18.0 \
  agc_smoothing_factor:=1500.0 agc_damping_factor:=30.0
```

Software (raw=true):
```bash
ros2 launch flir_ros_sync triage.launch.py raw:=true \
  agc_color_palette:=1 agc_gamma:=0.75 agc_max_gain:=1.8 agc_linear_percent:=30.0 \
  agc_tail_rejection:=2.0 agc_dde:=1.5 agc_detail_headroom:=18.0 \
  agc_smoothing_factor:=1400.0 agc_damping_factor:=20.0
```

#### (ii) General Person Inspection (balanced center detail, surroundings visible)

- Palette: WhiteHot (ID 0)
- Balanced gamma/gain, moderate damping; approximate center emphasis via linear_percent/plateau

Hardware AGC (raw=false, Ground UGV):
```bash
ros2 launch flir_ros_sync triage.launch.py raw:=false \
  deployment_profile:=ground agc_color_palette:=0 \
  agc_gamma:=0.80 agc_max_gain:=2.0 agc_linear_percent:=35.0 \
  agc_plateau:=8.0 agc_tail_rejection:=1.5 agc_dde:=1.3 \
  agc_detail_headroom:=18.0 agc_smoothing_factor:=1300.0 agc_damping_factor:=15.0
```

Air UAV variant:
```bash
ros2 launch flir_ros_sync triage.launch.py raw:=false \
  deployment_profile:=air agc_color_palette:=0 \
  agc_gamma:=0.80 agc_max_gain:=1.9 agc_linear_percent:=35.0 \
  agc_plateau:=9.0 agc_tail_rejection:=2.0 agc_dde:=1.3 \
  agc_detail_headroom:=18.0 agc_smoothing_factor:=1450.0 agc_damping_factor:=25.0
```

Software (raw=true):
```bash
ros2 launch flir_ros_sync triage.launch.py raw:=true \
  agc_color_palette:=0 agc_gamma:=0.80 agc_max_gain:=2.0 agc_linear_percent:=35.0 \
  agc_plateau:=8.0 agc_tail_rejection:=1.5 agc_dde:=1.3 \
  agc_detail_headroom:=18.0 agc_smoothing_factor:=1300.0 agc_damping_factor:=15.0
```

#### (iii) Close-up Person Inspection (minute motion: blinking, pulse)

- Palette: WhiteHot (ID 0). Optional: Ironbow (ID 4)
- Slightly lower gamma, more smoothing, slightly higher max_gain, emphasize local detail

Hardware AGC (raw=false, Ground UGV):
```bash
ros2 launch flir_ros_sync triage.launch.py raw:=false \
  deployment_profile:=ground agc_color_palette:=0 \
  agc_gamma:=0.70 agc_max_gain:=2.2 agc_linear_percent:=30.0 \
  agc_plateau:=7.0 agc_tail_rejection:=1.0 agc_dde:=1.4 \
  agc_detail_headroom:=20.0 agc_smoothing_factor:=1600.0 agc_damping_factor:=10.0
```

Optional alternate palette:
```bash
ros2 launch flir_ros_sync triage.launch.py raw:=false \
  deployment_profile:=ground agc_color_palette:=4 \
  agc_gamma:=0.70 agc_max_gain:=2.2 agc_linear_percent:=30.0 \
  agc_plateau:=7.0 agc_tail_rejection:=1.0 agc_dde:=1.4 \
  agc_detail_headroom:=20.0 agc_smoothing_factor:=1600.0 agc_damping_factor:=10.0
```

Software (raw=true):
```bash
ros2 launch flir_ros_sync triage.launch.py raw:=true \
  agc_color_palette:=0 agc_gamma:=0.70 agc_max_gain:=2.2 agc_linear_percent:=30.0 \
  agc_plateau:=7.0 agc_tail_rejection:=1.0 agc_dde:=1.4 \
  agc_detail_headroom:=20.0 agc_smoothing_factor:=1600.0 agc_damping_factor:=10.0
```

Notes:
- Center-ROI AGC is not currently exposed; settings above approximate center emphasis without new parameters.
- Palette IDs are consistent: 10 palettes (0–9) in both modes via `agc_color_palette`.

## Topics and Data Flow

### Hardware AGC Mode (`raw:=false` - **Recommended**)
- `/thermal/image` → **Hardware AGC + colormap** (8-bit mono8/BGR, 640x512 pixels)
- `/thermal/camera_info` → Camera calibration information
- `/thermal/core_temperature` → Camera core temperature in Celsius (std_msgs/Float32)
- **Benefits**: Native FLIR rendering, hardware acceleration, robust retry logic
- **Performance**: Minimal CPU usage, authentic FLIR colormap behavior

### Software Processing Mode (`raw:=true`)
- `/thermal/image` → 16-bit raw thermal data (sensor_msgs/Image, mono16, 640x512 pixels)
- `/thermal/image_rect` → 16-bit rectified thermal data (sensor_msgs/Image)
- `/thermal/image_processed` → **8-bit processed thermal image** (sensor_msgs/Image, BGR8 format)
- `/thermal/camera_info` → Camera calibration data (sensor_msgs/CameraInfo)
- `/thermal/core_temperature` → Camera core temperature in Celsius (std_msgs/Float32)

## Technical Implementation

### Hardware AGC Processing Pipeline (`raw:=false`)
1. **8-bit Processed Input** → Captured directly from FLIR Boson camera
2. **Hardware AGC** → Native Boson SDK processing with 11 configurable parameters
3. **Hardware Colormap** → Native SDK colormap rendering (10 professional colormaps)
4. **Robust Configuration** → Retry logic for reliable hardware parameter setting
5. **ROS Publishing** → 8-bit mono8/BGR format for visualization

### Software Processing Pipeline (`raw:=true`)
1. **16-bit Raw Input** → Captured directly from FLIR Boson camera
2. **Information-based AGC** → Advanced histogram equalization optimized for thermal data
3. **OpenMP Acceleration** → Parallel processing for enhanced performance
4. **Enhancement** → Optional contrast enhancement and rectification
5. **ROS Publishing** → 8-bit BGR8 format for visualization

### Processing Methods
- **Information-based**: Advanced histogram equalization (default)
- **Plateau-based**: Alternative processing method for specialized applications

## Performance Notes
- All colormaps run in real-time at 30 FPS
- Color processing adds minimal computational overhead
- BGR8 output format is compatible with RViz2 and OpenCV
- Percentile-based AGC provides robust temperature normalization

## Troubleshooting
- **No color output**: Ensure correct mode and parameter:
   - Hardware mode (`raw:=false`): colormap applied in-camera via `agc_color_palette:=<0-9>`
   - Software mode (`raw:=true`): colormap applied in node via `agc_color_palette:=<0-9>`
- **Wrong colors**: Check colormap spelling in launch command
- **Performance issues**: Disable enhancement (`enhance:=false`) if needed
- **Topic not found**: Verify `/thermal/image_processed` topic exists

---
### Hardware Colormaps with Expanded AGC Parameters
The system now supports **11 comprehensive AGC parameters** for fine-tuned control when using hardware colormaps (`raw:=false`):

```bash
# Person/Obstacle Detection Optimized (Default)
ros2 launch flir_ros_sync single_thermal_camera.launch.py raw:=false agc_color_palette:=1 \
  agc_gamma:=0.8 agc_max_gain:=2.0 agc_linear_percent:=30.0 agc_dde:=1.25 agc_detail_headroom:=16.0

# High Detail Imaging with Rainbow Colormap
ros2 launch flir_ros_sync single_thermal_camera.launch.py raw:=false agc_color_palette:=2 \
  agc_gamma:=0.6 agc_max_gain:=1.8 agc_plateau:=5.0 agc_dde:=1.5 agc_detail_headroom:=20.0 agc_smoothing_factor:=1500.0

# Low Noise Industrial Inspection
ros2 launch flir_ros_sync single_thermal_camera.launch.py raw:=false agc_color_palette:=4 \
  agc_gamma:=1.0 agc_max_gain:=1.5 agc_damping_factor:=10.0 agc_smoothing_factor:=2000.0 agc_detail_headroom:=12.0

# Fire Detection with Enhanced Edge Detail
ros2 launch flir_ros_sync single_thermal_camera.launch.py raw:=false agc_color_palette:=8 \
  agc_gamma:=0.7 agc_max_gain:=2.2 agc_dde:=1.4 agc_detail_headroom:=18.0 agc_tail_rejection:=3.0
```

### Key AGC Parameters for Colormap Optimization

| Parameter | Type | Colormap Impact | Recommended Values |
|-----------|------|----------------|-------------------|
| `agc_gamma` | float | Overall contrast and feature visibility | 0.6-0.8 for enhanced features |
| `agc_max_gain` | float | Detail preservation in colored regions | 1.8-2.2 for detailed colormaps |
| `agc_dde` | float | Edge sharpness in colored boundaries | 1.25-1.5 for crisp color transitions |
| `agc_detail_headroom` | float | Local edge detail in colored regions | 16.0-20.0 for enhanced detail |
| `agc_linear_percent` | float | Color gradient smoothness | 25.0-35.0 for smooth color mapping |
| `agc_smoothing_factor` | float | Color noise reduction | 1250.0-1500.0 for clean color rendering |


## Troubleshooting

### Hardware Mode (`raw:=false`) with Expanded AGC
- **No colormap applied**: Check retry logs, verify SDK communication, validate `agc_color_palette` parameter
 - **Wrong colors**: Verify colormap selection, check supported hardware colormaps (0–9)
- **Configuration failures**: Review retry mechanism logs, check camera firmware, validate AGC parameter ranges
- **Poor image quality**: Adjust `agc_gamma` (0.6-0.8), `agc_max_gain` (1.8-2.2), `agc_dde` (1.25-1.5)
- **Too much noise**: Increase `agc_smoothing_factor` (1500.0-2000.0), reduce `agc_max_gain` (1.5-1.8)
- **Lack of detail**: Increase `agc_dde` (1.4-1.6), increase `agc_detail_headroom` (18.0-22.0), adjust `agc_linear_percent` (25.0-35.0)
- **Unstable image**: Increase `agc_damping_factor` (10.0-20.0), reduce `agc_max_gain`

### Software Mode (`raw:=true`)
- **No color output**: Ensure raw mode enabled, check `/thermal/image_processed` topic
- **Flickering images**: Increase `agc_damping_factor` (0–100), e.g., `agc_damping_factor:=25.0`
- **Wrong colors**: Check palette ID and value (launch: `agc_color_palette:=<0-9>`), verify unified palette list
- **Performance issues**: Disable enhancement (`operation.processing.enhance:=false`) or reduce frame rate

### Expanded AGC Parameter Issues
- **Parameter out of range errors**: Check parameter ranges in launch validation logs
- **AGC not responding**: Verify camera initialization, check SDK communication
- **Inconsistent results**: Ensure parameter consistency, avoid conflicting settings
- **Validation failures**: Run `python3 scripts/validate_expanded_agc.py` to check configuration

### Qualitative Tuning Scenarios (Quick Guide)
- **Fast scene changes (panning, UAV motion)**: Raise `agc_damping_factor` for stability (20–40). If too sluggish, lower to 5–15. Keep `max_gain` moderate (1.8–2.5) to avoid pumping.
- **Low-contrast scenes (night, fog, uniform backgrounds)**: Increase `max_gain` (2.5–4.0) and `linear_percent` (35–55). Lower `gamma` slightly (0.6–0.8). Palettes: `ironbow(4)`, `lava(5)`, or `white_hot(0)`.
- **Hot-spot dominance (lamps, exhausts)**: Increase `tail_rejection` (1.5–3.0) and bias `outlier_cut_balance` toward the hot end (1.2–1.6). Consider lowering `max_gain` (1.5–2.2).
- **Detail emphasis (edges, small targets)**: Increase `detail_headroom` (16–24) and `dde` (1.2–1.6). Keep `smoothing_factor` moderate (1000–2000). Prefer `information_based` method.
- **Cluttered urban scenes (UGV/ground)**: `linear_percent` 35–45, `plateau` 8–12, `max_gain` 2.0–2.5, `gamma` 0.7–0.9. Palettes: `white_hot(0)`, `black_hot(1)`.
- **Wide temperature span (indoor→outdoor transitions)**: Slightly raise `agc_damping_factor` (15–30) and `tail_rejection` (1.0–2.0). Keep `max_gain` ≤2.5 to avoid harsh swings.
- **Sky/ground split (horizon in frame)**: Lower `linear_percent` (20–30) and raise `plateau` (10–15) to avoid over-stretch. Palettes: `arctic(6)` for cool background separation.
- **Through-glass/reflective surfaces**: Increase `tail_rejection` (2.0–3.0) to suppress reflections. Favor `white_hot(0)`/`black_hot(1)` to reduce misleading colors.

### Configuration Validation
```bash
# Validate expanded AGC configuration
python3 scripts/validate_expanded_agc.py

# Test specific parameter ranges
// removed invalid parameter example; use agc_color_palette instead
```

### General Issues
- **Topic not found**: Verify correct mode selection and topic names
- **Camera not detected**: Check USB connection and device permissions
- **Launch failures**: Review ROS2 parameter conflicts and launch file syntax
- **YAML configuration errors**: Use validation scripts to check parameter syntax and ranges
