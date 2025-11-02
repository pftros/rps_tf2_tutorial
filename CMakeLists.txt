cmake_minimum_required(VERSION 3.5)
project(rps_tf2_tutorial NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz urdf
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
