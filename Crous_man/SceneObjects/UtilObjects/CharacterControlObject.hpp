#ifndef CHARACTER_CONTROL_OBJECT_HPP
#define CHARACTER_CONTROL_OBJECT_HPP

#include "../../Scene.hpp"

class Texture;

class CharacterControlObject : public virtual SceneObject {
public:
    float speed;
    float sensitivity;

    float azimuth;

public:
    CharacterControlObject(float speed, float sensitivity);

    virtual void processInput(GLFWwindow *window, InputManager* inputManager, float deltaTime);
};

#endif