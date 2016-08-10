#define GLM_SWIZZLE
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/transform.hpp>
#include <gsl/gsl_multimin.h>
#include <gts.h>
#include <memory>
#include <stdio.h>
#include <vector>

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

#define VECTOR(vertex) \
    glm::vec3(vertex->p.x, vertex->p.y, vertex->p.z)

glm::vec2 vecAngleXZ;
glm::mat4 matModel;
glm::mat4 matView;
glm::mat4 matProj;
glm::mat4 matMvp;

double surfaceEquation(double x, double y)
{
    return pow(x * x + y * y, 5);
}

glm::vec3 nearestPoint(glm::vec3 p)
{
    std::shared_ptr<gsl_multimin_fminimizer> minimizer(
        gsl_multimin_fminimizer_alloc(
            gsl_multimin_fminimizer_nmsimplex2, 2),
        [] (gsl_multimin_fminimizer * m) {
            gsl_multimin_fminimizer_free(m);
        });

    gsl_multimin_function function;
    function.n = 2;
    function.params = &p;
    function.f = [] (const gsl_vector * in, void * params) -> double {
        const glm::dvec3 p = *reinterpret_cast<glm::vec3*>(params);
        const double x = gsl_vector_get(in, 0);
        const double y = gsl_vector_get(in, 1);
        const double z = surfaceEquation(x, y);
        return glm::length(p - glm::dvec3(x, y, z));
    };

    std::shared_ptr<gsl_vector> initVals(gsl_vector_alloc(2),
        [] (gsl_vector * v) { gsl_vector_free(v); });
    gsl_vector_set(initVals.get(), 0, p.x);
    gsl_vector_set(initVals.get(), 1, p.y);

    std::shared_ptr<gsl_vector> initSteps(gsl_vector_alloc(2),
        [] (gsl_vector * v) { gsl_vector_free(v); });
    gsl_vector_set(initSteps.get(), 0, 1e-3);
    gsl_vector_set(initSteps.get(), 1, 1e-3);

    gsl_multimin_fminimizer_set(
        minimizer.get(), &function, initVals.get(), initSteps.get());

    do {
        gsl_multimin_fminimizer_iterate(minimizer.get());
    } while(gsl_multimin_test_size(minimizer->size, 1e-7) == GSL_CONTINUE);

    const double x = gsl_vector_get(minimizer->x, 0);
    const double y = gsl_vector_get(minimizer->x, 1);
    const double z = surfaceEquation(x, y);
    return glm::vec3(x, y, z);
}

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
    vecAngleXZ[0] = glm::clamp(
        vecAngleXZ[0], -glm::half_pi<float>(), glm::half_pi<float>());

    matModel = glm::rotate(vecAngleXZ[0], glm::vec3(1.0f, 0.0f, 0.0f)) *
        glm::rotate(vecAngleXZ[1], glm::vec3(0.0f, 0.0f, 1.0f));

   oldXY = newXY;
}

gdouble refineCost(gpointer item, gpointer data)
{
    GtsEdge * e = GTS_EDGE(item);
    glm::vec3 v1 = VECTOR(e->segment.v1);
    glm::vec3 v2 = VECTOR(e->segment.v2);
    glm::dvec3 mv = (v1 + v2) / 2.0f;
    glm::dvec3 nv = nearestPoint(mv);
    return -glm::length(mv - nv);
}

GtsVertex * refineEdge(GtsEdge * e, GtsVertexClass * vcls, gpointer data)
{
    glm::vec3 v1 = VECTOR(e->segment.v1);
    glm::vec3 v2 = VECTOR(e->segment.v2);
    glm::dvec3 mv = (v1 + v2) / 2.0f;
    glm::dvec3 nv = nearestPoint(mv);
    return gts_vertex_new(vcls, nv.x, nv.y, nv.z);
}

gboolean refineStop(gdouble cost, guint nedge, gpointer data)
{
    return nedge > 1000;
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

    DEBUG("Building surface ...");
    GtsSurface * gtsSurface = gts_surface_new(
        gts_surface_class(),
        gts_face_class(),
        gts_edge_class(),
        gts_vertex_class());

    GtsVertexClass * vcls = gts_vertex_class();
    GtsVertex * v0 = gts_vertex_new(vcls, 0.0f, 0.0f, 0.0f);
    GtsVertex * v1 = gts_vertex_new(vcls,
        1.0f, 0.0f, surfaceEquation(1.0f, 0.0f));
    GtsVertex * v2 = gts_vertex_new(vcls,
        0.0f, 1.0f, surfaceEquation(0.0f, 1.0f));
    GtsVertex * v3 = gts_vertex_new(vcls,
        -1.0f, 0.0f, surfaceEquation(-1.0f, 0.0f));
    GtsVertex * v4 = gts_vertex_new(vcls,
        0.0f, -1.0f, surfaceEquation(0.0f, -1.0f));

    GtsEdgeClass * ecls = gts_edge_class();
    GtsEdge * e1 = gts_edge_new(ecls, v0, v1);
    GtsEdge * e2 = gts_edge_new(ecls, v0, v2);
    GtsEdge * e3 = gts_edge_new(ecls, v0, v3);
    GtsEdge * e4 = gts_edge_new(ecls, v0, v4);
    GtsEdge * e5 = gts_edge_new(ecls, v1, v2);
    GtsEdge * e6 = gts_edge_new(ecls, v2, v3);
    GtsEdge * e7 = gts_edge_new(ecls, v3, v4);
    GtsEdge * e8 = gts_edge_new(ecls, v4, v1);

    GtsFaceClass * fcls = gts_face_class();
    GtsFace * f1 = gts_face_new(fcls, e1, e2, e5);
    GtsFace * f2 = gts_face_new(fcls, e2, e3, e6);
    GtsFace * f3 = gts_face_new(fcls, e3, e4, e7);
    GtsFace * f4 = gts_face_new(fcls, e4, e1, e8);

    gts_surface_add_face(gtsSurface, f1);
    gts_surface_add_face(gtsSurface, f2);
    gts_surface_add_face(gtsSurface, f3);
    gts_surface_add_face(gtsSurface, f4);

    gts_surface_refine(gtsSurface,
        refineCost, nullptr,
        refineEdge, nullptr,
        refineStop, nullptr);

    std::vector<float> buffer;
    gts_surface_foreach_face(gtsSurface,
        [] (gpointer item, gpointer data) {
            std::vector<float> & buffer =
                *reinterpret_cast<std::vector<float>*>(data);
            GtsFace * face = GTS_FACE(item);
            GtsEdge * e1 = face->triangle.e1;
            GtsEdge * e2 = face->triangle.e2;
            GtsVertex * v1 = e1->segment.v1;
            GtsVertex * v2 = e1->segment.v2;
            GtsVertex * v3 = (e2->segment.v1 == v1 || e2->segment.v1 == v2)
                ? e2->segment.v2
                : e2->segment.v1;
            buffer.push_back(v1->p.x);
            buffer.push_back(v1->p.y);
            buffer.push_back(v1->p.z);
            buffer.push_back(v2->p.x);
            buffer.push_back(v2->p.y);
            buffer.push_back(v2->p.z);
            buffer.push_back(v3->p.x);
            buffer.push_back(v3->p.y);
            buffer.push_back(v3->p.z);
            return 0;
        }, &buffer);

    DEBUG("Creating vertex buffer ...");
    GLuint vertexBuffer = 0;
    VGL(glGenBuffers, 1, &vertexBuffer);
    VGL(glBindBuffer, GL_ARRAY_BUFFER, vertexBuffer);
    VGL(glBufferData, GL_ARRAY_BUFFER,
        buffer.size() * sizeof(float), buffer.data(), GL_STATIC_DRAW);
    VGL(glBindBuffer, GL_ARRAY_BUFFER, 0);

    DEBUG("Building vertex shader ...");
    const char * vertexShaderSource = R"(
        in vec3 pos;
        uniform mat4 mvp;
        void main()
        {
            gl_Position = mvp * vec4(pos, 1.0f);
        }
    )";
    GLuint vertexShader = VGL(glCreateShader, GL_VERTEX_SHADER);
    VGL(glShaderSource, vertexShader, 1, &vertexShaderSource, NULL);
    VGL(glCompileShader, vertexShader);

    DEBUG("Building fragment shader ...");
    const char * fragmentShaderSource = R"(
        void main()
        {
            gl_FragColor = vec4(1.0f, 1.0f, 1.0f, 1.0f);
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
        VGL(glVertexAttribPointer, 0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
        VGL(glEnableVertexAttribArray, 0);
        VGL(glBindBuffer, GL_ARRAY_BUFFER, 0);

        VGL(glDrawArrays, GL_TRIANGLES, 0, buffer.size() / 3);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}
