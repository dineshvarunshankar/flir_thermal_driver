cmake_minimum_required(VERSION 3.8)

# Use CUDA architectures policy if available; keep OLD on older CMake so empty architectures are allowed
if(POLICY CMP0104)
  if(CMAKE_VERSION VERSION_LESS "3.24")
    cmake_policy(SET CMP0104 OLD)
  else()
    cmake_policy(SET CMP0104 NEW)
  endif()
endif()

# Allow find_package to honor <PackageName>_ROOT variables (e.g., CUDAToolkit_ROOT)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

project(flir_ros_sync LANGUAGES C CXX)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Respect optional user hint to enable/disable CUDA; default ON so CUDA is used automatically when detected
option(WITH_CUDA "Build with CUDA if the toolkit is available" ON)

# --- CUDA auto-discovery (works with both FindCUDA and CUDAToolkit) ---
# Prefer system/user configuration; do not mutate environment or force RPATHs by default.
find_program(NVCC_EXECUTABLE
  NAMES nvcc
  HINTS /usr/local/cuda-12.6/bin /usr/local/cuda/bin /usr/local/cuda-12/bin
)
if(NVCC_EXECUTABLE)
  get_filename_component(_CUDA_BIN_DIR "${NVCC_EXECUTABLE}" DIRECTORY)
  get_filename_component(CUDA_HOME "${_CUDA_BIN_DIR}/.." ABSOLUTE)
  # Hints for legacy FindCUDA and modern CUDAToolkit
  if(NOT DEFINED CUDAToolkit_ROOT)
    set(CUDAToolkit_ROOT     "${CUDA_HOME}" CACHE PATH "CUDA root" FORCE)
  endif()
  if(NOT DEFINED CUDA_TOOLKIT_ROOT_DIR)
    set(CUDA_TOOLKIT_ROOT_DIR "${CUDA_HOME}" CACHE PATH "CUDA root" FORCE)
  endif()
  message(STATUS "Detected CUDA at: ${CUDA_HOME} (nvcc: ${NVCC_EXECUTABLE})")
else()
  message(STATUS "nvcc not found on PATH; CUDA will be disabled unless specified explicitly")
endif()

# Find OpenMP for parallel processing
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP found: ${OpenMP_CXX_VERSION}")
    add_compile_options(${OpenMP_CXX_FLAGS})
    add_definitions(-DWITH_OPENMP=1)
else()
    message(STATUS "OpenMP not found - building without parallel processing")
    add_definitions(-DWITH_OPENMP=0)
endif()

# Force find_package to use the ROS-provided OpenCV to resolve version conflicts
set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(std_msgs REQUIRED)

# CUDA toolchain and integration - Modern CMake approach
if(WITH_CUDA)
    # Enable CUDA language support
    enable_language(CUDA)
    
    # Import modern CUDA targets (e.g. CUDA::cudart) if available
    find_package(CUDAToolkit QUIET)

    if(CMAKE_CUDA_COMPILER)
        message(STATUS "CUDA found: ${CMAKE_CUDA_COMPILER_VERSION}")
        if(TARGET CUDA::cudart)
          message(STATUS "CUDAToolkit imported targets are available (CUDA::cudart)")
        else()
          message(WARNING "CUDAToolkit imported targets not found; will attempt to link cudart by name")
        endif()
        
        add_compile_definitions(WITH_CUDA=1)
        
        # Build GPU mapper as a CUDA library
        add_library(gpu_agc_lib STATIC src/gpu_agc_mapper.cu)
        target_compile_definitions(gpu_agc_lib PRIVATE WITH_CUDA=1)
        target_include_directories(gpu_agc_lib PRIVATE 
            ${CMAKE_CURRENT_SOURCE_DIR}/include
        )
        
        # Set CUDA properties before setting compile options
        set_target_properties(gpu_agc_lib PROPERTIES 
            POSITION_INDEPENDENT_CODE ON
            CUDA_SEPARABLE_COMPILATION ON
        )
        
        # For CMake < 3.24, set CUDA architectures compatible with CUDA 12.8
        if(CMAKE_VERSION VERSION_LESS "3.24")
          # Use supported architectures for CUDA 12.8 (remove old/unsupported ones)
          set_property(TARGET gpu_agc_lib PROPERTY CUDA_ARCHITECTURES "52;60;61;70;75;80;86")
          message(STATUS "CMake ${CMAKE_VERSION} detected. Using CUDA architectures: 52;60;61;70;75;80;86")
        else()
          # Use supported architectures for CUDA 12.8 (remove old/unsupported ones)
          set_property(TARGET gpu_agc_lib PROPERTY CUDA_ARCHITECTURES "52;60;61;70;75;80;86")
          message(STATUS "CMake ${CMAKE_VERSION} detected. Using CUDA architectures: 52;60;61;70;75;80;86")
        endif()
        
        # Comprehensive warning suppression for NVCC-generated code
        target_compile_options(gpu_agc_lib PRIVATE
            # Suppress warnings for all languages
            $<$<COMPILE_LANGUAGE:C>:-Wno-pedantic>
            $<$<COMPILE_LANGUAGE:C>:-Wno-unused-parameter>
            $<$<COMPILE_LANGUAGE:CXX>:-Wno-pedantic>
            $<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-parameter>
            # NVCC host compiler warnings (passed through -Xcompiler)
            $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-Wno-pedantic>
            $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-Wno-gnu-line-marker>
            $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-Wno-unknown-pragmas>
            $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-Wno-unused-parameter>
            $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-Wno-sign-compare>
            $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-Wno-unused-variable>
            # NVCC device compiler warnings
            $<$<COMPILE_LANGUAGE:CUDA>:-w>
        )

        # Explicitly link the CUDA runtime (mirrors prior working setup)
        if(TARGET CUDA::cudart)
          target_link_libraries(gpu_agc_lib PUBLIC CUDA::cudart)
        else()
          # Fallback: locate libcudart by name
          find_library(CUDART_LIB cudart HINTS ${CUDA_HOME}/lib64 ${CMAKE_CUDA_TOOLKIT_ROOT_DIR}/lib64 ${CUDAToolkit_ROOT}/lib64)
          if(CUDART_LIB)
            target_link_libraries(gpu_agc_lib PUBLIC ${CUDART_LIB})
            message(STATUS "Found CUDA runtime: ${CUDART_LIB}")
          endif()
        endif()
        
        list(APPEND LINK_LIBRARIES gpu_agc_lib)
        
        # Optional RPATH embedding (enabled by default to mirror historically working setup)
        option(WITH_CUDA_RPATH "Embed CUDA lib64 RPATH in built targets" OFF)
        if(WITH_CUDA_RPATH)
            if(DEFINED CUDA_HOME)
                set(CMAKE_BUILD_RPATH "${CUDA_HOME}/lib64")
                set(CMAKE_INSTALL_RPATH "${CUDA_HOME}/lib64")
            elseif(DEFINED CMAKE_CUDA_TOOLKIT_ROOT_DIR)
                set(CMAKE_BUILD_RPATH "${CMAKE_CUDA_TOOLKIT_ROOT_DIR}/lib64")
                set(CMAKE_INSTALL_RPATH "${CMAKE_CUDA_TOOLKIT_ROOT_DIR}/lib64")
            endif()
            set(CMAKE_SKIP_BUILD_RPATH FALSE)
            set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
            set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
        endif()
    else()
        message(STATUS "CUDA compiler not found - building without GPU acceleration")
        add_compile_definitions(WITH_CUDA=0)
    endif()
else()
    message(STATUS "CUDA disabled by user - building without GPU acceleration")
    add_compile_definitions(WITH_CUDA=0)
endif()

# Include directories
include_directories(
  include
)

# CUDA includes are already handled in the CUDA detection section

# Define the Boson SDK source directory
set(BOSON_SDK_CLIENT_FILES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/script/Boson_SDK/ClientFiles_C)
set(BOSON_SDK_FSLP_FILES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/script/Boson_SDK/FSLP_Files/src)
set(BOSON_SDK_FSLP_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/script/Boson_SDK/FSLP_Files/src/inc)
set(BOSON_SDK_FSLP_LINUX_DIR ${CMAKE_CURRENT_SOURCE_DIR}/script/Boson_SDK/FSLP_Files/src/linux)

# Add SDK source files
# set(BOSON_SDK_SRCS
#   ${BOSON_SDK_SRC_DIR}/Client_API.c
#   ${BOSON_SDK_SRC_DIR}/EnumTypes.h
#   ${BOSON_SDK_SRC_DIR}/ReturnCodes.h
#   ${BOSON_SDK_SRC_DIR}/Client_Packager.c
# )
file(GLOB BOSON_SDK_SRCS
  ${BOSON_SDK_CLIENT_FILES_DIR}/*.c
  ${BOSON_SDK_FSLP_FILES_DIR}/*.c
  ${BOSON_SDK_FSLP_INC_DIR}/*.h
  ${BOSON_SDK_FSLP_LINUX_DIR}/*.c
  ${BOSON_SDK_FSLP_LINUX_DIR}/*.h
)

set_source_files_properties(${BOSON_SDK_SRCS} PROPERTIES LANGUAGE C)

# Create a static library for BOSON_SDK
add_library(Boson_SDK STATIC
  ${BOSON_SDK_SRCS}
)

# Include directories for BOSON_SDK
target_include_directories(Boson_SDK PUBLIC
  ${BOSON_SDK_CLIENT_FILES_DIR}
  ${BOSON_SDK_FSLP_INC_DIR}
  ${BOSON_SDK_FSLP_LINUX_DIR}
)

# Suppress noisy warnings from auto-generated vendor SDK code (GCC only)
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
  target_compile_options(Boson_SDK PRIVATE
    -Wno-uninitialized
    -Wno-maybe-uninitialized
    -Wno-unused-variable
    -Wno-unused-parameter
    -Wno-unused-but-set-variable
    -Wno-pedantic
    -Wno-array-parameter
    -Wno-strict-aliasing
    -Wno-misleading-indentation
    -Wno-implicit-function-declaration
    -Wno-sizeof-pointer-memaccess
    -Wno-sign-compare
    -Wno-unused-result
  )
endif()

# Library sources
set(LIBRARY_SOURCES
  include/flir_ros_sync/flir_ros_sync.h
  include/flir_ros_sync/fd_guard.h
  include/flir_ros_sync/gpu_agc_mapper.h
  include/flir_ros_sync/thermal_processor.h
  src/flir_ros_sync.cpp
  src/thermal_processor.cpp
  src/fast_thermal_core.c
)

# CUDA sources are handled via cuda_add_library above

# Library
add_library(${PROJECT_NAME} SHARED ${LIBRARY_SOURCES})

# CUDA dependencies are handled through LINK_LIBRARIES

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${BOSON_SDK_CLIENT_FILES_DIR}
  ${BOSON_SDK_FSLP_INC_DIR}
  ${BOSON_SDK_FSLP_LINUX_DIR}
)

# CUDA properties are handled by cuda_add_library

# Link against libraries (append to existing CUDA libraries if present)
list(APPEND LINK_LIBRARIES
  Boson_SDK
  pthread
)

# CUDA linking and compile definitions are handled by the CUDA integration section above

target_link_libraries(${PROJECT_NAME} ${LINK_LIBRARIES} ${OpenCV_LIBRARIES})

# Register component
rclcpp_components_register_nodes(${PROJECT_NAME} "flir_ros_sync::FlirRos")

# Add CUDA thermal processor executable

# Add thermal processor executable
add_executable(thermal_processor
  src/thermal_processor_main.cpp
)

ament_target_dependencies(thermal_processor
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  camera_info_manager
  image_geometry
)

# Link thermal processor executable to main library
target_link_libraries(thermal_processor ${PROJECT_NAME} ${OpenCV_LIBRARIES})
target_include_directories(thermal_processor PRIVATE include)

# Link OpenMP if available
if(OpenMP_CXX_FOUND)
    target_link_libraries(thermal_processor OpenMP::OpenMP_CXX)
    target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()

# Link CUDA library to main target when CUDA is enabled
if(WITH_CUDA AND CMAKE_CUDA_COMPILER)
  target_link_libraries(${PROJECT_NAME} gpu_agc_lib)
endif()

# Dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  cv_bridge
  camera_info_manager
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  image_geometry
  std_msgs
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install executables
install(TARGETS
  thermal_processor
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install helper Python script only if present
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/script/trigger_ffc.py)
  install(PROGRAMS
    script/trigger_ffc.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()




# Install Boson SDK module
install(
  DIRECTORY script/Boson_SDK
  DESTINATION lib/${PROJECT_NAME}
)

# Install data
install(DIRECTORY data/
    DESTINATION share/${PROJECT_NAME}/data
)

# Install config (including new thermal processor params)
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

# Install targets
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

# Install launch files if any
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()