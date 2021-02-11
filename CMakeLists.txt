cmake_minimum_required(VERSION 3.5)
project(mrs_lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions("-Wall")
add_definitions("-Wextra")
add_definitions("-g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# please, NEVER commit those alternative flags with specific overrides of optimization
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3") # because normal release is just -O2
# please, really do not commit your temporary custom flags :-)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

# include OpenCV
find_package(OpenCV REQUIRED)

# include PCL
find_package(PCL REQUIRED COMPONENTS)

# include include
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                       param loader                         |
## --------------------------------------------------------------

add_library(MrsLibParamLoader SHARED
  src/param_loader/param_loader.cpp
)
ament_export_libraries(MrsLibParamLoader)

add_executable(param_loader_example
  src/param_loader/example.cpp
)
ament_target_dependencies(param_loader_example
  rclcpp
)

## --------------------------------------------------------------
## |                        scope timer                         |
## --------------------------------------------------------------

add_library(MrsLibScopeTimer SHARED
  src/scope_timer/scope_timer.cpp
)
ament_export_libraries(MrsLibScopeTimer)

## --------------------------------------------------------------
## |                         geometry                           |
## --------------------------------------------------------------

add_library(MrsLibGeometry SHARED
  src/geometry/misc.cpp src/geometry/cyclic.cpp src/geometry/shapes.cpp
)
ament_export_libraries(MrsLibGeometry)

## --------------------------------------------------------------
## |                           utils                            |
## --------------------------------------------------------------

add_library(MrsLibUtils SHARED
  src/utils/utils.cpp
)
ament_export_libraries(MrsLibUtils)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  # the executables to be exported have to be listed here
  TARGETS
  param_loader_example
  DESTINATION lib/${PROJECT_NAME})

# so that includes are visible for whomever includes mrs_lib
ament_export_include_directories(include)
install(
  # the libraries to be exported for other packages have to be listed here
  TARGETS
  MrsLibParamLoader
  MrsLibScopeTimer
  MrsLibGeometry
  MrsLibUtils
  LIBRARY  DESTINATION lib
  ARCHIVE  DESTINATION lib
  RUNTIME  DESTINATION bin
  INCLUDES DESTINATION include
)

# build the package using the above settings
ament_package()
