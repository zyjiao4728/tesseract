cmake_minimum_required(VERSION 3.5.0)
project(tesseract_motion_planners VERSION 0.1.0 LANGUAGES CXX)

find_package(ompl REQUIRED)
find_package(trajopt REQUIRED)
find_package(tesseract REQUIRED)
find_package(console_bridge REQUIRED)
find_package(Boost REQUIRED)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_11 CXX_FEATURE_FOUND)

# Create interface for core
add_library(${PROJECT_NAME}_core INTERFACE)
target_link_libraries(${PROJECT_NAME}_core INTERFACE tesseract::tesseract console_bridge)
target_compile_options(${PROJECT_NAME}_core INTERFACE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_core INTERFACE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_core INTERFACE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_core INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_core SYSTEM INTERFACE
    ${EIGEN3_INCLUDE_DIRS})

# Trajopt Planner
add_library(${PROJECT_NAME}_trajopt SHARED src/trajopt/trajopt_motion_planner.cpp)
target_link_libraries(${PROJECT_NAME}_trajopt PUBLIC ${Boost_LIBRARIES} ${PROJECT_NAME}_core trajopt::trajopt trajopt::trajopt_utils trajopt::trajopt_sco)
target_compile_options(${PROJECT_NAME}_trajopt PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_trajopt PUBLIC -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_trajopt PUBLIC cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_trajopt PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_trajopt SYSTEM INTERFACE
    ${Boost_INCLUDE_DIRS})

# Trajopt Freespace Planner
add_library(${PROJECT_NAME}_trajopt_freespace SHARED src/trajopt/trajopt_freespace_planner.cpp)
target_link_libraries(${PROJECT_NAME}_trajopt_freespace PUBLIC ${PROJECT_NAME}_trajopt)
target_compile_options(${PROJECT_NAME}_trajopt_freespace PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_trajopt_freespace PUBLIC -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_trajopt_freespace PUBLIC cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_trajopt_freespace PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

# Trajopt Array Planner
add_library(${PROJECT_NAME}_trajopt_array SHARED src/trajopt/trajopt_array_planner.cpp)
target_link_libraries(${PROJECT_NAME}_trajopt_array PUBLIC ${PROJECT_NAME}_trajopt)
target_compile_options(${PROJECT_NAME}_trajopt_array PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_trajopt_array PUBLIC -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_trajopt_array PUBLIC cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_trajopt_array PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

# OMPL Planning Interface
add_library(${PROJECT_NAME}_ompl SHARED
  src/ompl/chain_ompl_interface.cpp
  src/ompl/conversions.cpp
  src/ompl/continuous_motion_validator.cpp
)
target_link_libraries(${PROJECT_NAME}_ompl PUBLIC ${PROJECT_NAME}_core ${OMPL_LIBRARIES})
target_compile_options(${PROJECT_NAME}_ompl PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_ompl PUBLIC -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_ompl PUBLIC cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_ompl PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_ompl SYSTEM PUBLIC
    ${OMPL_INCLUDE_DIRS})

list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME}_core)
list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME}_trajopt)
list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME}_trajopt_freespace)
list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME}_trajopt_array)
list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME}_ompl)

install(TARGETS ${PACKAGE_LIBRARIES} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets NAMESPACE tesseract:: DESTINATION lib/cmake/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Create cmake config files
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})

export(EXPORT ${PROJECT_NAME}-targets FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)

if (ENABLE_TESTS)
  enable_testing()
  add_custom_target(run_tests ALL
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIGURATION> -V)

  add_subdirectory(test)
endif()
