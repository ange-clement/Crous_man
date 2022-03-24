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

#include "KeyboardControlObject.hpp"

#include "../../Transform.hpp"
#include "../../InputManager.hpp"

KeyboardControlObject::KeyboardControlObject(float speed) : SceneObject() {
    this->speed = speed;
}

void KeyboardControlObject::processInput(GLFWwindow *window, InputManager* inputManager, float deltaTime) {
    float translationAmount = this->speed * deltaTime;

    glm::vec3 rdirection = glm::vec3(1.0, 0.0, 0.0);
    glm::vec3 udirection = glm::vec3(0.0, 1.0, 0.0);
    glm::vec3 fdirection = glm::vec3(0.0, 0.0, 1.0);

    if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
        translationAmount*=4;
    }
    if (glfwGetKey(window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS) {
        translationAmount/=8;
    }

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        this->transform->translation += translationAmount * fdirection;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        this->transform->translation -= translationAmount * fdirection;

    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        this->transform->translation += translationAmount * rdirection;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        this->transform->translation -= translationAmount * rdirection;

    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
        this->transform->translation -= translationAmount * udirection;
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
        this->transform->translation += translationAmount * udirection;
    
    SceneObject::processInput(window, inputManager, deltaTime);
}