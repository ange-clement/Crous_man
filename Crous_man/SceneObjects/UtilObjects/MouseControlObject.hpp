#ifndef MOUSE_CONTROL_OBJECT_HPP
#define MOUSE_CONTROL_OBJECT_HPP

#include "../../Scene.hpp"

class Texture;

class MouseControlObject : public virtual SceneObject {
public:
    float sensibility;

    float azimuth, zenith;

public:
    MouseControlObject(float sensibility);

    virtual void processInput(GLFWwindow *window, InputManager* inputManager, float deltaTime);
};

#endif