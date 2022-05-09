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
#include <Crous_man/Util.hpp>
#include <Crous_man/ECS/EntityManager.hpp>
#include <Crous_man/ECS/Bitmap.hpp>
#include <Crous_man/ECS/Entity.hpp>

#include "../RigidBody.hpp"

#include "CrousManController.hpp"

#include "../Collider.hpp"
#include "../Destructible.hpp"



CrousManControllerSystem::CrousManControllerSystem() : ComponentSystem() {
    requiredComponentsBitmap = new Bitmap({ SystemIDs::CrousManControllerID, SystemIDs::RigidBodyID });
}

CrousManControllerSystem::~CrousManControllerSystem() {

}

void CrousManControllerSystem::initialize(unsigned short i, unsigned short entityID) {
    CrousManController* crous = getCrousManController(i);
    crous->maxSpeed = 1.0f;
    crous->acceleration = crous->maxSpeed * 4.0f;
    crous->sensitivity = 0.16f;
    crous->azimuth = 0.0;
    crous->zenith = 0.0;

    crous->initialRotatingPos = EntityManager::instance->entities[crous->rotatingCenterForCamera->id]->transform->translation;

    crous->laserCooldown = new Cooldown(3.0f);

    crous->rb = &EntityManager::instance->rigidBodyComponents[EntityManager::instance->getComponentId(SystemIDs::RigidBodyID, entityID)];

    if (colliderSystem == NULL) {
        colliderSystem = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID]);
    }
    if (destructibleSystem == NULL) {
        destructibleSystem = dynamic_cast<DestructibleSystem*>(EntityManager::instance->systems[SystemIDs::DestructibleID]);
    }



    //TODO supprimer
    RigidBodySystem* rbSystem = dynamic_cast<RigidBodySystem*>(EntityManager::instance->systems[SystemIDs::RigidBodyID]);
    rbSystem->setGravity(glm::vec3(0.0f));
}

void CrousManControllerSystem::update(unsigned short i, unsigned short entityID) {
    //Get data
    CrousManController* crous = getCrousManController(i);
    RigidBody* rb = crous->rb;

    Entity* crousEntity = EntityManager::instance->entities[entityID];
    Transform* tr = crousEntity->transform;
    Transform* wtr = crousEntity->worldTransform;

    Entity* rotatingEntity = EntityManager::instance->entities[crous->rotatingCenterForCamera->id];
    Transform* rotatingTr = rotatingEntity->transform;
    Transform* rotatingWTr = rotatingEntity->worldTransform;

    Entity* cameraTargetEntity = EntityManager::instance->entities[crous->cameraTarget->id];
    Transform* cameraTargetTr = cameraTargetEntity->transform;
    Transform* cameraTargetWTr = cameraTargetEntity->worldTransform;
    
    Entity* cameraEntity = EntityManager::instance->entities[crous->camera->id];
    Transform* cameraTr = cameraEntity->transform;
    Transform* cameraWTr = cameraEntity->worldTransform;

    Entity* saucisseEntity = EntityManager::instance->entities[crous->saucisseEntity->id];
    Transform* saucisseEntityTr = saucisseEntity->transform;

    float accelerationAmount = crous->acceleration;
    glm::vec3 appliedAcceleration = glm::vec3(0.0f);
    float currentMaxSpeed = crous->maxSpeed;


    // process input
    glm::vec3 r = rotatingTr->getRight();
    glm::vec3 rdirection = glm::normalize(glm::vec3(r.x, 0.0, r.z));
    glm::vec3 udirection = glm::vec3(0.0, 1.0, 0.0);
    glm::vec3 f = rotatingTr->getForward();
    glm::vec3 fdirection = glm::normalize(glm::vec3(f.x, 0.0, f.z));

    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
        currentMaxSpeed *= 1.8f;
    }
    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS) {
        currentMaxSpeed *= 0.5f;
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

    // Laser : raycast
    if (glfwGetMouseButton(InputManager::instance->window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS) {
        Ray* ray = new Ray(cameraWTr->translation, cameraWTr->getForward());
        std::vector<RaycastResult*> rayResult = colliderSystem->rayCastAll(*ray);
        RaycastResult* closest = NULL;
        float minDistance = FLT_MAX;
        float distance;
        glm::vec3 cameraToResult;
        for (unsigned int i = 0, size = rayResult.size(); i < size; i++) {
            if (rayResult[i] != NULL && !EntityManager::instance->hasComponent(SystemIDs::CrousManControllerID, rayResult[i]->entityIDCollid)) {
                cameraToResult = rayResult[i]->point - cameraWTr->translation;
                distance = glm::dot(cameraToResult, cameraToResult);
                if (distance < minDistance) {
                    minDistance = distance;
                    closest = rayResult[i];
                }
            }
        }

        glm::vec3 laserHit;
        if (closest != NULL) {
            distance = sqrt(distance);
            laserHit = closest->point;

            if (EntityManager::instance->hasComponent(SystemIDs::DestructibleID, closest->entityIDCollid)) {
                unsigned int destructibleID = EntityManager::instance->getComponentId(SystemIDs::DestructibleID, closest->entityIDCollid);
                destructibleSystem->destroyAmount(destructibleID, 1.0f * InputManager::instance->deltaTime);
            }
        }
        else {
            distance = 200.0f;
            laserHit = cameraWTr->translation + distance * cameraWTr->getForward();
        }
        float laserSize = sin(InputManager::instance->lastFrame * 15.0f)*1.0f+2.0f;
        glm::vec3 crousToHit = laserHit - tr->translation;
        crous->laserEntity->transform->translation = tr->translation + crousToHit * 0.5f;
        crous->laserEntity->transform->lookAt(laserHit);
        crous->laserEntity->transform->scaling = glm::vec3(laserSize, laserSize, distance * 0.5f);
        crous->laserEntity->setActiveRecursive(true);
    }
    else {
        crous->laserEntity->setActiveRecursive(false);
    }


    // apply input

    rb->addAcceleration(appliedAcceleration);

    if (!mooved) {
        if (glm::dot(rb->velocity, rb->velocity) > 0.01f)
            rb->addAcceleration(-accelerationAmount * 2.0f * glm::normalize(rb->velocity));
        else
            rb->velocity = glm::vec3(0.0f);
    }


    // process mouse movement
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


        glm::vec3 diferenceVector = (tr->translation + crous->initialRotatingPos) - rotatingTr->translation;
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

        // Raycast between rotating and the camera to move it in front of the nearest wall

        glm::vec3 rotatingToCameraTarget = glm::normalize(cameraTargetWTr->translation - rotatingWTr->translation);
        Ray* ray = new Ray(rotatingWTr->translation, rotatingToCameraTarget);
        std::vector<RaycastResult*> rayResult = colliderSystem->rayCastAll(*ray);
        RaycastResult* closest = NULL;
        float minDistance = FLT_MAX;
        float distance;
        glm::vec3 rotatingToResult;
        for (unsigned int i = 0, size = rayResult.size(); i < size; i++) {
            if (rayResult[i] != NULL && !EntityManager::instance->hasComponent(SystemIDs::CrousManControllerID, rayResult[i]->entityIDCollid)) {
                rotatingToResult = rayResult[i]->point - cameraTargetWTr->translation;
                distance = glm::dot(rotatingToResult, rotatingToResult);
                if (distance < minDistance) {
                    minDistance = distance;
                    closest = rayResult[i];
                }
            }
        }

        if (closest != NULL) {
            distance = std::min((float)sqrt(distance), crous->maxCameraDistance);
        }
        else {
            distance = crous->maxCameraDistance;
        }
        cameraTr->translation = rotatingWTr->translation + rotatingToCameraTarget * distance;
        cameraTr->lookAtDirection(rotatingToCameraTarget);
    }
}

void CrousManControllerSystem::addEntityComponent() {
    EntityManager::instance->crousManControllerComponents.push_back(CrousManController());
}

CrousManController* CrousManControllerSystem::getCrousManController(unsigned short i) {
    return &EntityManager::instance->crousManControllerComponents[i];
}


