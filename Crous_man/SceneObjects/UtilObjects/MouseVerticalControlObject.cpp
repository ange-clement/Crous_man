#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "../../Util.hpp"
#include "../../Scene.hpp"

#include "MouseVerticalControlObject.hpp"

#include "../../Transform.hpp"
#include "../../InputManager.hpp"

MouseVerticalControlObject::MouseVerticalControlObject(float sensibility) : SceneObject() {
    this->sensibility = sensibility;
    this->zenith = 0;
}

void MouseVerticalControlObject::processInput(GLFWwindow *window, InputManager* inputManager, float deltaTime) {
    float rotationAmount = this->sensibility * deltaTime;
    float rotationY = inputManager->mouseOffsetY * rotationAmount;

    zenith  += rotationY;

    if(zenith > 1.5f)
        zenith = 1.5f;
    if(zenith < -1.5f)
        zenith = -1.5f;

    this->transform->rotation.setRotation(-zenith, glm::vec3(1.0, 0.0, 0.0));
    
    SceneObject::processInput(window, inputManager, deltaTime);
}