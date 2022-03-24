#ifndef MOVABLE_CAMERA_HPP
#define MOVABLE_CAMERA_HPP

#include "../Scene.hpp"

enum CameraMode {
    ORBITAL,
    FREE
};

class MovableCamera : public virtual Camera {
public:
    CameraMode mode;

    float camera_distance = 5.0;
    glm::vec3 camera_target   = glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 camera_up       = glm::vec3(0.0f, 1.0f,  0.0f);
    glm::vec3 camera_forward  = glm::vec3(0.0f, 0.0f, -1.0f);

    unsigned int SCR_WIDTH = 1024;
    unsigned int SCR_HEIGHT = 768;

    float fov = 45;
    float ratio = 4.0/3.0;
    float minRange = 0.01;
    float maxRange = 1000.0;
    glm::mat4 view;
    glm::mat4 projection;

    bool disableMouse = false;
    bool freeMouse = true;
    bool canSwitchFreeMouse = true;
    bool firstMouse = true;
    double lastX, lastY;
    float azimuth = 1.5;
    float zenith = 0.;

public:
    
    MovableCamera();

    void updateMVP();

    glm::vec3 processCameraMovement(GLFWwindow *window, glm::vec3 startPos, float deltaTime);

    void processInput(GLFWwindow *window, float deltaTime);

    void mouse_callback(GLFWwindow* window, double xpos, double ypos, float deltaTime);

    void scroll_callback(float scroll_distance);
};

#endif