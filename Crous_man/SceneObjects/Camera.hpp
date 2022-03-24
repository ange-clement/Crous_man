#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "../Scene.hpp"

class Camera : public virtual SceneObject {
public:
    unsigned int SCR_WIDTH = 1024;
    unsigned int SCR_HEIGHT = 768;

    float fov = 45;
    float ratio = 4.0/3.0;
    float minRange = 0.01;
    float maxRange = 1000.0;
    glm::mat4 view;
    glm::mat4 projection;

public:
    
    Camera();

    void updateMVP();

    virtual void processInput(GLFWwindow *window, InputManager* inputManager, float deltaTime);
};

#endif