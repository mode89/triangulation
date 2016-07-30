#define GLM_SWIZZLE
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/transform.hpp>
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

glm::vec2 vecAngleXZ;
glm::mat4 matModel;
glm::mat4 matView;
glm::mat4 matProj;
glm::mat4 matMvp;

void onResize(GLFWwindow * window, int width, int height)
{
    VGL(glViewport, 0, 0, width, height);
    matProj = glm::perspective(
        0.8f, static_cast<float>(width) / height, 1.0f, 100.0f);
}

void onMouseMove(GLFWwindow * window, double x, double y)
{
    static glm::vec2 oldXY = glm::vec2(x, y);
    glm::vec2 newXY = glm::vec2(x, y);

    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
        vecAngleXZ += (newXY - oldXY).yx() / 100.0f;

    matModel = glm::rotate(vecAngleXZ[0], glm::vec3(1.0f, 0.0f, 0.0f)) *
        glm::rotate(vecAngleXZ[1], glm::vec3(0.0f, 0.0f, 1.0f));

   oldXY = newXY;
}

int main()
{
    DEBUG("Initializing GLFW ...");
    glfwInit();
    glfwWindowHint(GLFW_SAMPLES, 16);

    DEBUG("Creating window ...");
    GLFWwindow * window = glfwCreateWindow(
        640, 480, "Triangulation", NULL, NULL);
    glfwSetFramebufferSizeCallback(window, onResize);
    glfwSetCursorPosCallback(window, onMouseMove);
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

    const char * vertexShaderSource = R"(
        in vec2 pos;
        uniform mat4 mvp;
        void main()
        {
            gl_Position = mvp * vec4(pos, 0.0f, 1.0f);
        }
    )";
    GLuint vertexShader = VGL(glCreateShader, GL_VERTEX_SHADER);
    VGL(glShaderSource, vertexShader, 1, &vertexShaderSource, NULL);
    VGL(glCompileShader, vertexShader);

    const char * fragmentShaderSource = R"(
        void main()
        {
            gl_FragColor = vec4(1.0f, 0.0f, 0.0f, 1.0f);
        }
    )";
    GLuint fragmentShader = VGL(glCreateShader, GL_FRAGMENT_SHADER);
    VGL(glShaderSource, fragmentShader, 1, &fragmentShaderSource, NULL);
    VGL(glCompileShader, fragmentShader);

    GLuint shaderProgram = VGL(glCreateProgram);
    VGL(glAttachShader, shaderProgram, vertexShader);
    VGL(glAttachShader, shaderProgram, fragmentShader);
    VGL(glLinkProgram, shaderProgram);

    GLint uniMvp = VGL(glGetUniformLocation, shaderProgram, "mvp");

    matView = glm::lookAt(
        glm::vec3(0.0f, -5.0f, 5.0f),
        glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, 0.0f, 1.0f));

    DEBUG("Running ...");
    while (!glfwWindowShouldClose(window))
    {
        VGL(glClearColor, 0.0f, 0.0f, 0.3f, 1.0f);
        VGL(glClear, GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        VGL(glUseProgram, shaderProgram);

        matMvp = matProj * matView * matModel;
        VGL(glUniformMatrix4fv,
            uniMvp, 1, GL_FALSE, glm::value_ptr(matMvp));

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
