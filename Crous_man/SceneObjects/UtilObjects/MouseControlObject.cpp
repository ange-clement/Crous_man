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

#include "MouseControlObject.hpp"

#include "../../Transform.hpp"
#include "../../InputManager.hpp"

MouseControlObject::MouseControlObject(float sensibility) : SceneObject() {
    this->sensibility = sensibility;
    this->azimuth = 0;
    this->zenith = 0;
}

void MouseControlObject::processInput(GLFWwindow *window, InputManager* inputManager, float deltaTime) {
    float rotationAmount = this->sensibility * deltaTime;
    float rotationX = inputManager->mouseOffsetX * rotationAmount;
    float rotationY = inputManager->mouseOffsetY * rotationAmount;

    azimuth += rotationX;
    zenith  += rotationY;

    if(zenith > 1.5f)
        zenith = 1.5f;
    if(zenith < -1.5f)
        zenith = -1.5f;

    this->transform->rotation.setRotation(azimuth, glm::vec3(0.0, 1.0, 0.0));
    this->transform->rotation.combineRotation(-zenith, glm::vec3(1.0, 0.0, 0.0));
    
    SceneObject::processInput(window, inputManager, deltaTime);
}