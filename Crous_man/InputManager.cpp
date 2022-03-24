#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "InputManager.hpp"

InputManager::InputManager() {
    firstMouseMouve = true;
    lastMouseX = 0;
    lastMouseY = 0;
}
    
void InputManager::processInput(GLFWwindow *window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
    
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

void InputManager::scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    scroll_distance -= yoffset;

    if (scroll_distance < 1.0) {
        scroll_distance = 1.0;
    }
}

void InputManager::framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    SCR_WIDTH = width;
    SCR_HEIGHT = height;

    glViewport(0, 0, width, height);
}