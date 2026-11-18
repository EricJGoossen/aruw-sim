cmake_minimum_required(VERSION 3.8)
project(sentry_description)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  install(DIRECTORY config launch urdf visualizer
    DESTINATION share/${PROJECT_NAME}
  )
  install(DIRECTORY urdf/meshes
    DESTINATION share/${PROJECT_NAME}
  )
  ament_package()
else()
  find_package(catkin REQUIRED)
  catkin_package()
  foreach(dir config launch urdf visualizer)
    install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
  endforeach(dir)
  install(DIRECTORY urdf/meshes/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)
endif()
