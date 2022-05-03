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

#include "SimpleMovementPlayer.hpp"

SimpleMovementPlayerSystem::SimpleMovementPlayerSystem() : ComponentSystem() {
    requiredComponentsBitmap = new Bitmap({ SystemIDs::SimplePlayerControllerID });
}

SimpleMovementPlayerSystem::~SimpleMovementPlayerSystem() {}


void SimpleMovementPlayerSystem::initialize(unsigned short i, unsigned short entityID) {
    SimpleMovementPlayer* fc = getSimpleMovementController(i);
    fc->speed = 40.0f;
}

void SimpleMovementPlayerSystem::update(unsigned short i, unsigned short entityID) {
    SimpleMovementPlayer* sp = getSimpleMovementController(i);
    Transform* tr = EntityManager::instance->entities[entityID]->transform;

    float translationAmount = sp->speed * InputManager::instance->deltaTime;

    glm::vec3 r = tr->getRight();
    glm::vec3 rdirection = glm::normalize(glm::vec3(r.x, 0.0, r.z));
    glm::vec3 udirection = glm::vec3(0.0, 1.0, 0.0);
    glm::vec3 f = tr->getForward();
    glm::vec3 fdirection = glm::normalize(glm::vec3(f.x, 0.0, f.z));

    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
        translationAmount *= 4;
    }
    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS) {
        translationAmount /= 8;
    }

    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_UP) == GLFW_PRESS) {
        tr->translation += translationAmount * fdirection;
    }

    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_DOWN) == GLFW_PRESS) {
        tr->translation -= translationAmount * fdirection;
    }

    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_LEFT) == GLFW_PRESS) {
        tr->translation -= translationAmount * rdirection;
    }

    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
        tr->translation += translationAmount * rdirection;
    }

    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS) {
        tr->translation -= translationAmount * udirection;
    }
    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_ENTER) == GLFW_PRESS) {
        tr->translation += translationAmount * udirection;
    }
}

void SimpleMovementPlayerSystem::addEntityComponent() {
    EntityManager::instance->simplePlayerControllerComponents.push_back(SimpleMovementPlayer());
}

SimpleMovementPlayer* SimpleMovementPlayerSystem::getSimpleMovementController(unsigned short i) {
    return &EntityManager::instance->simplePlayerControllerComponents[i];
}