

add_library(basics SHARED
    basics/basics.cpp
    basics/config.cpp
    basics/yaml.cpp
    basics/eigen_funcs.cpp
    basics/opencv_funcs.cpp
)

add_library(display SHARED
    display/pcl_display.cpp
    display/pcl_display_lib.cpp
)

add_library(geometry SHARED
    geometry/camera.cpp
    geometry/feature_match.cpp
    geometry/epipolar_geometry.cpp
    geometry/motion_estimation.cpp
)

add_library( optimization SHARED
    optimization/g2o_ba.cpp
)

add_library( vo SHARED
    vo/frame.cpp
    vo/vo.cpp
    vo/vo_addFrame.cpp
    vo/vo_io.cpp
    vo/map.cpp
    vo/mappoint.cpp
    vo/vo_commons.cpp
)

add_library( ros1 SHARED
    ros1/posestamped_publisher.cpp
)

target_link_libraries( basics
    ${THIRD_PARTY_LIBS} 
)

target_link_libraries( display
    ${THIRD_PARTY_LIBS} basics
)

target_link_libraries( geometry
    ${THIRD_PARTY_LIBS} basics
)

target_link_libraries( optimization
    ${THIRD_PARTY_LIBS} basics
)

target_link_libraries( vo
    ${THIRD_PARTY_LIBS} geometry basics optimization
)

target_link_libraries( ros1
    ${roscpp_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
)

# add_executable(main main.cpp)
# target_link_libraries( main
#     vo geometry
# )
