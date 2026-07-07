cmake_minimum_required(VERSION 3.16)
project(lite_urdf)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf mjcf mappings launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  find_file(BAR_AMENT_FLAKE8_CONFIG ament_flake8.ini
            PATHS ${CMAKE_CURRENT_SOURCE_DIR}/..
                  ${CMAKE_CURRENT_SOURCE_DIR}/../..
            NO_DEFAULT_PATH)
  if(BAR_AMENT_FLAKE8_CONFIG)
    set(ament_cmake_flake8_CONFIG_FILE "${BAR_AMENT_FLAKE8_CONFIG}")
  endif()
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
