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

#include "Camera.hpp"

#include "../Transform.hpp"
#include "../Mesh.hpp"
#include "../InputManager.hpp"

Camera::Camera() : SceneObject() {
    ratio = this->SCR_WIDTH/(float)this->SCR_HEIGHT;
}

void Camera::updateMVP() {
    view = glm::lookAt(
        this->worldTransform->translation,
        this->worldTransform->applyToPoint(glm::vec3(0.0, 0.0, 1.0)),
        glm::vec3(0.0, 1.0, 0.0)
    );
    //view = this->worldTransform->inverse()->toMat4();

    projection = glm::perspective(
        this->fov,
        this->ratio,
        this->minRange, this->maxRange
    );
}


void Camera::processInput(GLFWwindow *window, InputManager* inputManager, float deltaTime) {
    this->SCR_WIDTH = inputManager->SCR_WIDTH;
    this->SCR_HEIGHT = inputManager->SCR_HEIGHT;
    ratio = this->SCR_WIDTH/(float)this->SCR_HEIGHT;
}