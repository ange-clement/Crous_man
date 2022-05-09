#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "ECS/EntityManager.hpp"
#include "Components/Camera.hpp"

#include "InputManager.hpp"

InputManager* InputManager::instance = NULL;

InputManager::InputManager() {
    if (InputManager::instance == NULL) {
        InputManager::instance = this;
        firstMouseMouve = true;
        lastMouseX = 0;
        lastMouseY = 0;
        scroll_distance = 0;
        lastFrame = 0.0f;
        disableMouse = false;
    } else {
        std::cerr << "Error : cannot instanciate two InputManager" << std::endl;
    }
}
    
void InputManager::update(GLFWwindow *window) {
    if (this->window != window) {
        this->window = window;
    }
    float currentFrame = glfwGetTime();
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;
    
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
    
    if (!disableMouse && glfwGetKey(window, GLFW_KEY_TAB) == GLFW_PRESS) {
        disableMouse = true;
        glfwSetCursorPos(window, SCR_WIDTH/2, SCR_WIDTH/2);
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }
    else if (disableMouse && glfwGetKey(window, GLFW_KEY_TAB) == GLFW_RELEASE) {
        disableMouse = false;
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    }

    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);
    if (firstMouseMouve) {
        lastMouseX = xpos;
        lastMouseY = ypos;
        firstMouseMouve = false;
    }
    
    mouseOffsetX = lastMouseX - xpos;
    mouseOffsetY = lastMouseY - ypos;
    lastMouseX = xpos;
    lastMouseY = ypos;
}

void InputManager::initialize(GLFWwindow *window) {
    lastFrame = glfwGetTime();
    deltaTime = FLT_EPSILON;
}

void InputManager::scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    scroll_distance -= yoffset;

    if (scroll_distance < 1.0) {
        scroll_distance = 1.0;
    }
}

void InputManager::framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    if (abs((int)SCR_WIDTH - width) > 50 || abs((int)SCR_HEIGHT - height) > 50) {
        SCR_WIDTH = width;
        SCR_HEIGHT = height;

        glViewport(0, 0, width, height);
        dynamic_cast<CameraSystem*>(EntityManager::instance->systems[SystemIDs::CameraID])->screenCamera->updateWidthHeight(width, height);
    }
}