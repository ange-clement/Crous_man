#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <common/ray.hpp>
#include <common/entityPool.hpp>
#include <common/cooldown.hpp>

#include <Crous_man/Transform.hpp>

#include <Crous_man/InputManager.hpp>
#include <Crous_man/ECS/EntityManager.hpp>
#include <Crous_man/ECS/Bitmap.hpp>
#include <Crous_man/ECS/Entity.hpp>

#include "../RigidBody.hpp"

#include "CrousManController.hpp"

#include "../Collider.hpp"



CrousManControllerSystem::CrousManControllerSystem() : ComponentSystem() {
    requiredComponentsBitmap = new Bitmap({ SystemIDs::CrousManControllerID, SystemIDs::RigidBodyID });
}

CrousManControllerSystem::~CrousManControllerSystem() {

}

void CrousManControllerSystem::initialize(unsigned short i, unsigned short entityID) {
    CrousManController* fc = getCrousManController(i);
    fc->maxSpeed = 1.0f;
    fc->acceleration = fc->maxSpeed * 2.0f;
    fc->sensitivity = 0.16f;
    fc->azimuth = 0.0;
    fc->zenith = 0.0;

    fc->laserCooldown = new Cooldown(3.0f);

    fc->rb = &EntityManager::instance->rigidBodyComponents[EntityManager::instance->getComponentId(SystemIDs::RigidBodyID, entityID)];

    if (colliderSystem == NULL) {
        colliderSystem = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID]);
    }


    //TODO supprimer
    RigidBodySystem* rbSystem = dynamic_cast<RigidBodySystem*>(EntityManager::instance->systems[SystemIDs::RigidBodyID]);
    rbSystem->setGravity(glm::vec3(0.0f));
}

void CrousManControllerSystem::update(unsigned short i, unsigned short entityID) {
    CrousManController* crous = getCrousManController(i);
    RigidBody* rb = crous->rb;
    Transform* tr = EntityManager::instance->entities[entityID]->transform;
    Transform* wtr = EntityManager::instance->entities[entityID]->worldTransform;
    Transform* rotatingTr = EntityManager::instance->entities[crous->rotatingCenterForCamera->id]->transform;
    Transform* rotatingWTr = EntityManager::instance->entities[crous->rotatingCenterForCamera->id]->worldTransform;
    Transform* cameraTargetTr = EntityManager::instance->entities[crous->cameraTarget->id]->transform;
    Transform* saucisseEntityTr = EntityManager::instance->entities[crous->saucisseEntity->id]->transform;

    float accelerationAmount = crous->acceleration;
    glm::vec3 appliedAcceleration = glm::vec3(0.0f);
    float currentMaxSpeed = crous->maxSpeed;

    glm::vec3 r = rotatingTr->getRight();
    glm::vec3 rdirection = glm::normalize(glm::vec3(r.x, 0.0, r.z));
    glm::vec3 udirection = glm::vec3(0.0, 1.0, 0.0);
    glm::vec3 f = rotatingTr->getForward();
    glm::vec3 fdirection = glm::normalize(glm::vec3(f.x, 0.0, f.z));

    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
        currentMaxSpeed *= 4;
    }
    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS) {
        currentMaxSpeed /= 8;
    }

    bool mooved = false;
    if (glm::dot(rb->velocity, fdirection) < currentMaxSpeed && glfwGetKey(InputManager::instance->window, GLFW_KEY_W) == GLFW_PRESS) {
        appliedAcceleration += accelerationAmount * fdirection;
        mooved = true;
    }
    if (glm::dot(rb->velocity, fdirection) > -currentMaxSpeed && glfwGetKey(InputManager::instance->window, GLFW_KEY_S) == GLFW_PRESS) {
        appliedAcceleration += -accelerationAmount * fdirection;
        mooved = true;
    }
    if (glm::dot(rb->velocity, rdirection) < currentMaxSpeed && glfwGetKey(InputManager::instance->window, GLFW_KEY_A) == GLFW_PRESS) {
        appliedAcceleration += accelerationAmount * rdirection;
        mooved = true;
    }
    if (glm::dot(rb->velocity, rdirection) > -currentMaxSpeed && glfwGetKey(InputManager::instance->window, GLFW_KEY_D) == GLFW_PRESS) {
        appliedAcceleration += -accelerationAmount * rdirection;
        mooved = true;
    }

    rb->addAcceleration(appliedAcceleration);

    if (!mooved) {
        if (glm::dot(rb->velocity, rb->velocity) > 0.01f)
            rb->addAcceleration(-accelerationAmount * 2.0f * glm::normalize(rb->velocity));
        else
            rb->velocity = glm::vec3(0.0f);
    }


    if (!InputManager::instance->disableMouse) {
        float rotationAmount = crous->sensitivity * InputManager::instance->deltaTime;
        float rotationX = InputManager::instance->mouseOffsetX * rotationAmount;
        float rotationY = InputManager::instance->mouseOffsetY * rotationAmount;

        crous->azimuth += rotationX;
        crous->zenith += rotationY;

        if (crous->zenith > 1.5f)
            crous->zenith = 1.5f;
        if (crous->zenith < -1.5f)
            crous->zenith = -1.5f;

        if (mooved) {
            crous->lastMoovedAzimuth = crous->azimuth;
            crous->lastMoovedZenith = crous->zenith;
        }


        glm::vec3 diferenceVector = tr->translation - rotatingTr->translation;
        rotatingTr->translation += diferenceVector * 0.1f;
        rotatingTr->rotation.setRotation(crous->azimuth, glm::vec3(0.0, 1.0, 0.0));
        rotatingTr->rotation.combineRotation(-crous->zenith, glm::vec3(1.0, 0.0, 0.0));
        
        diferenceVector = tr->translation + glm::vec3(0.0, 7.0, 0.0) - saucisseEntityTr->translation;
        saucisseEntityTr->translation += diferenceVector * 0.2f;

        if (glm::dot(rb->velocity, rb->velocity) > 0.0f) {
            glm::vec3 velocityFdirection = glm::normalize(glm::vec3(rb->velocity.x, 0.0, rb->velocity.z));
            glm::vec3 target = tr->translation + velocityFdirection;
            tr->rotation.lookAt(tr->translation, target, glm::vec3(0.0f, 1.0f, 0.0f));
            saucisseEntityTr->rotation.lookAt(tr->translation, target, glm::vec3(0.0f, 1.0f, 0.0f));
        }

    }
}

void CrousManControllerSystem::addEntityComponent() {
    EntityManager::instance->crousManControllerComponents.push_back(CrousManController());
}

CrousManController* CrousManControllerSystem::getCrousManController(unsigned short i) {
    return &EntityManager::instance->crousManControllerComponents[i];
}


