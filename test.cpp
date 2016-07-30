#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <stdio.h>

#define DEBUG(...) \
    printf(__VA_ARGS__); \
    printf("\n");

#define ERROR(...) \
    DEBUG(__VA_ARGS__); \
    throw 0;

#define VGL(func, ...) \
    func(__VA_ARGS__); \
    if (glGetError() != GL_NONE) \
    { \
        ERROR("Failed in " #func "()"); \
    }

int main()
{
    DEBUG("Initializing GLFW ...");
    glfwInit();
    glfwWindowHint(GLFW_SAMPLES, 16);

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

    DEBUG("Creating vertex buffer ...");
    const float vertices[] = {
        -1.0f, -1.0f,
        -1.0f,  1.0f,
         1.0f, -1.0f,
         1.0f,  1.0f
    };
    GLuint vertexBuffer = 0;
    VGL(glGenBuffers, 1, &vertexBuffer);
    VGL(glBindBuffer, GL_ARRAY_BUFFER, vertexBuffer);
    VGL(glBufferData, GL_ARRAY_BUFFER,
        sizeof(vertices), vertices, GL_STATIC_DRAW);
    VGL(glBindBuffer, GL_ARRAY_BUFFER, 0);

    DEBUG("Running ...");
    while (!glfwWindowShouldClose(window))
    {
        VGL(glClearColor, 0.0f, 0.0f, 0.3f, 1.0f);
        VGL(glClear, GL_COLOR_BUFFER_BIT);

        VGL(glPolygonMode, GL_FRONT_AND_BACK, GL_LINE);

        VGL(glBindBuffer, GL_ARRAY_BUFFER, vertexBuffer);
        VGL(glVertexAttribPointer, 0, 2, GL_FLOAT, GL_FALSE, 0, NULL);
        VGL(glEnableVertexAttribArray, 0);
        VGL(glBindBuffer, GL_ARRAY_BUFFER, 0);

        VGL(glDrawArrays, GL_TRIANGLE_STRIP, 0, 4);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}
