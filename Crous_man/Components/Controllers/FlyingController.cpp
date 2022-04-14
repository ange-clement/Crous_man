#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <Crous_man/InputManager.hpp>
#include <Crous_man/ECS/EntityManager.hpp>
#include <Crous_man/ECS/Bitmap.hpp>
#include <Crous_man/ECS/Entity.hpp>

#include <Crous_man/Transform.hpp>

#include "FlyingController.hpp"

FlyingControllerSystem::FlyingControllerSystem() : ComponentSystem(){
    requiredComponentsBitmap = new Bitmap({SystemIDs::FlyingControllerID});
}

FlyingControllerSystem::~FlyingControllerSystem() {

}

void FlyingControllerSystem::initialize(unsigned short i, unsigned short entityID) {
    FlyingController* fc = getFlyingController(i);
    fc->speed = 40.0f;
    fc->sensitivity = 0.12f;
    fc->azimuth = 0.0;
    fc->zenith = 0.0;
}

void FlyingControllerSystem::update(unsigned short i, unsigned short entityID) {
    FlyingController* fc = getFlyingController(i);
    Transform* tr = EntityManager::instance->entities[entityID]->transform;

    float translationAmount = fc->speed * InputManager::instance->deltaTime;

    glm::vec3 r = tr->getRight();
    glm::vec3 rdirection = glm::normalize(glm::vec3(r.x, 0.0, r.z));
    glm::vec3 udirection = glm::vec3(0.0, 1.0, 0.0);
    glm::vec3 f = tr->getForward();
    glm::vec3 fdirection = glm::normalize(glm::vec3(f.x, 0.0, f.z));

    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
        translationAmount*=4;
    }
    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS) {
        translationAmount/=8;
    }

    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_W) == GLFW_PRESS)
        tr->translation += translationAmount * fdirection;
    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_S) == GLFW_PRESS)
        tr->translation -= translationAmount * fdirection;

    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_A) == GLFW_PRESS)
        tr->translation += translationAmount * rdirection;
    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_D) == GLFW_PRESS)
        tr->translation -= translationAmount * rdirection;

    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
        tr->translation -= translationAmount * udirection;
    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_SPACE) == GLFW_PRESS)
        tr->translation += translationAmount * udirection;

    
    if (!InputManager::instance->disableMouse) {
        float rotationAmount = fc->sensitivity * InputManager::instance->deltaTime;
        float rotationX = InputManager::instance->mouseOffsetX * rotationAmount;
        float rotationY = InputManager::instance->mouseOffsetY * rotationAmount;

        fc->azimuth += rotationX;
        fc->zenith  += rotationY;

        if(fc->zenith > 1.5f)
            fc->zenith = 1.5f;
        if(fc->zenith < -1.5f)
            fc->zenith = -1.5f;

        tr->rotation.setRotation(fc->azimuth, glm::vec3(0.0, 1.0, 0.0));
        tr->rotation.combineRotation(-fc->zenith, glm::vec3(1.0, 0.0, 0.0));
    }
}

void FlyingControllerSystem::addEntityComponent() {
    EntityManager::instance->flyingControllerComponents.push_back(FlyingController());
}

FlyingController* FlyingControllerSystem::getFlyingController(unsigned short i) {
    return &EntityManager::instance->flyingControllerComponents[i];
}