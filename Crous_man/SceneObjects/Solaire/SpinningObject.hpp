#ifndef SPINNING_OBJECT_HPP
#define SPINNING_OBJECT_HPP

#include "../../Scene.hpp"

class Texture;

class SpinningObject : public virtual SceneObject {
public:
    float speed;

public:
    SpinningObject(float speed);

    virtual void processInput(GLFWwindow *window, InputManager* inputManager, float deltaTime);
};

#endif