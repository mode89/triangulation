project(TRIANGULATION)
cmake_minimum_required(VERSION 3.0)

# Dependencies

    find_library(GLFW_LIBRARY glfw)

    find_package(GLEW)
    find_package(OpenGL)

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
            ${OPENGL_LIBRARIES}
    )
