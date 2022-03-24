#ifndef MOUSE_VERTICAL__CONTROL_OBJECT_HPP
#define MOUSE_VERTICAL__CONTROL_OBJECT_HPP

#include "../../Scene.hpp"

class Texture;

class MouseVerticalControlObject : public virtual SceneObject {
public:
    float sensibility;

    float zenith;

public:
    MouseVerticalControlObject(float sensibility);

    virtual void processInput(GLFWwindow *window, InputManager* inputManager, float deltaTime);
};

#endif