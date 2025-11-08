cmake_minimum_required(VERSION 3.5)
project(tf2_tutorial_robot NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz urdf
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
