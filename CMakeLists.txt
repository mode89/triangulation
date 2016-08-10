project(TRIANGULATION)
cmake_minimum_required(VERSION 3.0)

# Configuration

    set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# Dependencies

    find_library(GLFW_LIBRARY glfw)

    find_package(GLEW)
    find_package(OpenGL)

    add_subdirectory(gsl)
    add_subdirectory(gts)

# Targets

    add_executable(triangulation-test
        test.cpp
    )

# Flags

    target_compile_options(triangulation-test
        PRIVATE
            -std=c++11
    )

# Linkage

    target_link_libraries(triangulation-test
        PRIVATE
            ${GLEW_LIBRARIES}
            ${GLFW_LIBRARY}
            gsl
            gts
            ${OPENGL_LIBRARIES}
    )
