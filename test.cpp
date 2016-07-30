#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <stdio.h>

#define DEBUG(...) \
    printf(__VA_ARGS__); \
    printf("\n");

#define VGL(func, ...) \
    func(__VA_ARGS__); \
    if (glGetError() != GL_NONE) \
    { \
        DEBUG("Failed in " #func "()"); \
    }

int main()
{
    DEBUG("Initializing GLFW ...");
    glfwInit();

    DEBUG("Creating window ...");
    GLFWwindow * window = glfwCreateWindow(
        640, 480, "Triangulation", NULL, NULL);
    glfwSetFramebufferSizeCallback(window,
        [] (GLFWwindow * window, int width, int height) {
            VGL(glViewport, 0, 0, width, height);
        }
    );
    glfwMakeContextCurrent(window);

    DEBUG("Initializing GLEW ...");
    glewInit();

    DEBUG("Running ...");
    while (!glfwWindowShouldClose(window))
    {
        VGL(glClearColor, 0.0f, 0.0f, 0.3f, 1.0f);
        VGL(glClear, GL_COLOR_BUFFER_BIT);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}
