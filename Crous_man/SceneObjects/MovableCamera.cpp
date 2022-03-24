#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "../Util.hpp"
#include "../../external/image_ppm.h"
#include "../Scene.hpp"

#include "MovableCamera.hpp"

#include "../Transform.hpp"
#include "../Mesh.hpp"

MovableCamera::MovableCamera() : Camera() {
    this->mode = CameraMode::FREE;
}

void MovableCamera::updateMVP() {
    // View matrix : camera/view transformation lookat() utiliser this->transform->translation camera_target camera_up
    view = glm::lookAt(
        this->worldTransform->translation,
        this->worldTransform->applyToPoint(camera_forward),
        glm::vec3(0.0, 1.0, 0.0)
    );
    //glm::mat4 view = this->worldTransform->toMat4();

    // Projection matrix : 45 Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
    ratio = this->SCR_WIDTH/(float)this->SCR_HEIGHT;
    projection = glm::perspective(
        this->fov,
        this->ratio,
        this->minRange, this->maxRange
    );
}

glm::vec3 MovableCamera::processCameraMovement(GLFWwindow *window, glm::vec3 startPos, float deltaTime) {
    //Camera zoom in and out
    glm::vec3 camera_right = glm::normalize(cross(camera_up, camera_forward));

    float cameraSpeed = 10 * deltaTime;

    glm::vec3 fdirection = glm::normalize(glm::vec3(camera_forward.x, 0.0, camera_forward.z));
    glm::vec3 rdirection = glm::normalize(glm::vec3(camera_right.x, 0.0, camera_right.z));
    glm::vec3 udirection = glm::normalize(glm::vec3(0.0, 1.0, 0.0));

    glm::vec3 newPosition = startPos;

    if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
        cameraSpeed*=4;
    }
    if (glfwGetKey(window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS) {
        cameraSpeed/=8;
    }

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        newPosition += cameraSpeed * fdirection;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        newPosition -= cameraSpeed * fdirection;

    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        newPosition += cameraSpeed * rdirection;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        newPosition -= cameraSpeed * rdirection;
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
        newPosition -= cameraSpeed * udirection;
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
        newPosition += cameraSpeed * udirection;
    
    return newPosition;
}

void MovableCamera::processInput(GLFWwindow *window, float deltaTime) {
    if (canSwitchFreeMouse && glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_PRESS) {
        freeMouse = !freeMouse;
        canSwitchFreeMouse = false;
        if (!freeMouse) {
            camera_target = this->transform->translation + 5.0f*camera_forward;
            camera_distance = 5.0f;
        }
    }
    if (glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_RELEASE) {
        canSwitchFreeMouse = true;
    }

    if (!disableMouse && glfwGetKey(window, GLFW_KEY_TAB) == GLFW_PRESS) {
        disableMouse = true;
        glfwSetCursorPos(window, SCR_WIDTH/2, SCR_WIDTH/2);
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }
    else if (disableMouse && glfwGetKey(window, GLFW_KEY_TAB) == GLFW_RELEASE) {
        disableMouse = false;
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    }

    if (!disableMouse) {
        if (freeMouse) {
            glm::vec3 newPosition = processCameraMovement(window, this->transform->translation, deltaTime);
            this->transform->translation = newPosition;
        }
        else {
            glm::vec3 newPosition = processCameraMovement(window, camera_target, deltaTime);
            camera_target = newPosition;
            glm::vec3 direction = camera_target - this->transform->translation;
            camera_forward = glm::normalize(direction);
            camera_distance = glm::length(direction);
        }

        
        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);
        mouse_callback(window, xpos, ypos, deltaTime);
    }

    transform->translation = this->transform->translation;

    for (size_t i = 0, size = this->childrens.size(); i < size; i++) {
        this->childrens[i]->processInput(window, deltaTime);
    }
}

void MovableCamera::mouse_callback(GLFWwindow* window, double xpos, double ypos, float deltaTime) {
    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos;
    lastX = xpos;
    lastY = ypos;


    float sensitivity = 0.12f * deltaTime;
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    azimuth += xoffset;
    zenith  += yoffset;

    if(zenith > 1.5f)
        zenith = 1.5f;
    if(zenith < -1.5f)
        zenith = -1.5f;

    glm::vec3 direction;
    direction.x = cos(azimuth) * cos(zenith);
    direction.y = sin(zenith);
    direction.z = sin(azimuth) * cos(zenith);

    if (freeMouse) {
        camera_forward = direction;
    }
    else {
        this->transform->translation = camera_target + direction * camera_distance;
    }
}

void MovableCamera::scroll_callback(float scroll_distance) {
    if (!freeMouse) {
        this->transform->translation = camera_target + glm::normalize(camera_target - this->transform->translation) * scroll_distance * scroll_distance;
        camera_distance = scroll_distance;
    }
}