#ifndef KEYBOARD_CONTROL_OBJECT_HPP
#define KEYBOARD_CONTROL_OBJECT_HPP

#include "../../Scene.hpp"

class Texture;

class KeyboardControlObject : public virtual SceneObject {
public:
    float speed;

public:
    KeyboardControlObject(float speed);

    virtual void processInput(GLFWwindow *window, InputManager* inputManager, float deltaTime);
};

#endif