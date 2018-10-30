cmake_minimum_required(VERSION 3.5)
project(raspicam)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



if(${CMAKE_SYSTEM_PROCESSOR} MATCHES arm*)
  FIND_LIBRARY(mmal_LIBS NAMES mmal PATHS /opt/vc/lib)
  FIND_LIBRARY(mmalcore_LIBS NAMES mmal_core PATHS /opt/vc/lib)
  FIND_LIBRARY(mmalutil_LIBS NAMES mmal_util PATHS /opt/vc/lib)
  FIND_LIBRARY(mmalvcclient_LIBS NAMES mmal_vc_client PATHS /opt/vc/lib)
  IF( (NOT mmal_LIBS ) OR (NOT mmalutil_LIBS) OR (NOT mmalcore_LIBS) OR (NOT mmalvcclient_LIBS) )
      MESSAGE(FATAL_ERROR "Could not find mmal libraries")
  ENDIF()

  FIND_LIBRARY(vcos_LIBS NAMES vcos PATHS /opt/vc/lib)
  IF( (NOT vcos_LIBS ))
      MESSAGE(FATAL_ERROR "Could not find vcos library")
  ENDIF()

  FIND_LIBRARY(bcmhost_LIBS NAMES bcm_host PATHS /opt/vc/lib)
  IF( (NOT bcmhost_LIBS ))
      MESSAGE(FATAL_ERROR "Could not find bcm_host library")
  ENDIF()

  FIND_LIBRARY(vchostif_LIBS NAMES vchostif PATHS /opt/vc/lib)
  IF( (NOT vchostif_LIBS ))
      MESSAGE(FATAL_ERROR "Could not find vchostif library")
  ENDIF()

  set(RPI_LIBRARIES ${RPI_LIBRARIES} 
  ${bcmhost_LIBS}
  ${vcos_LIBS} 
  ${mmal_LIBS} 
  ${mmalcore_LIBS} 
  ${mmalutil_LIBS}  
  ${mmalvcclient_LIBS} 
  ${vchostif_LIBS} 
  )
ENDIF()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  /opt/vc/include
  /opt/vc/include/interface/vcos/pthreads
  /opt/vc/include/interface/vmcs_host/linux
)


add_executable(raspicam_node
  src/raspicam.cpp
  src/RaspiCamControl.cpp
)

target_link_libraries(raspicam_node
  ${RPI_LIBRARIES}
)


ament_target_dependencies(raspicam_node rclcpp std_srvs sensor_msgs ${RPI_LIBRARIES})

install(TARGETS raspicam_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
