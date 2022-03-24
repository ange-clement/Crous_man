#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "../../../common/meshGenerator.hpp"

#include "../../Util.hpp"
#include "../../../external/image_ppm.h"
#include "../../Scene.hpp"

#include "SpinningObject.hpp"

#include "../../Transform.hpp"

SpinningObject::SpinningObject(float speed) : SceneObject() {
    this->speed = speed;
}

void SpinningObject::processInput(GLFWwindow *window, InputManager* inputManager, float deltaTime) {
    this->transform->rotation.combineRotation(this->speed * deltaTime, glm::vec3(0.0, 1.0, 0.0));

    SceneObject::processInput(window, inputManager, deltaTime);
}