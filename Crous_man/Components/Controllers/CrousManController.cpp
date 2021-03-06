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
#include <Crous_man/SoundManager.hpp>
#include <Crous_man/Util.hpp>
#include <Crous_man/ECS/EntityManager.hpp>
#include <Crous_man/ECS/Bitmap.hpp>
#include <Crous_man/ECS/Entity.hpp>

#include "../RigidBody.hpp"

#include "CrousManController.hpp"

#include "../Collider.hpp"
#include "../Destructible.hpp"

#define DEBUG_CONTROLLER false

CrousManControllerSystem::CrousManControllerSystem() : ComponentSystem() {
    requiredComponentsBitmap = new Bitmap({ SystemIDs::CrousManControllerID, SystemIDs::RigidBodyID });
}

CrousManControllerSystem::~CrousManControllerSystem() {

}

void CrousManControllerSystem::updateOnCollide(unsigned short i, unsigned short entityID, const std::vector<ColliderResult*>& collisionResults) {
    for (unsigned int c = 0, size = collisionResults.size(); c < size; c++) {
        if (EntityManager::instance->hasComponent(SystemIDs::DestructibleID, collisionResults[c]->entityCollidID)) {
            unsigned int destructibleID = EntityManager::instance->getComponentId(SystemIDs::DestructibleID, collisionResults[c]->entityCollidID);
            destructibleSystem->destroyAmount(destructibleID, .5f * InputManager::instance->deltaTime);
        }
    }
}

void CrousManControllerSystem::initialize(unsigned short i, unsigned short entityID) {
    CrousManController* crous = getCrousManController(i);
    crous->maxSpeed = 50.0f;
    crous->acceleration = crous->maxSpeed * 4.0f;
    crous->sensitivity = 0.16f;
    crous->azimuth = 0.0;
    crous->zenith = 0.0;

    crous->initialRotatingPos = EntityManager::instance->entities[crous->rotatingCenterForCamera->id]->transform->translation;

    crous->laserSoundCooldown = new Cooldown(0.3f);

    crous->rb = &EntityManager::instance->rigidBodyComponents[EntityManager::instance->getComponentId(SystemIDs::RigidBodyID, entityID)];

    if (colliderSystem == NULL) {
        colliderSystem = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID]);
    }
    if (destructibleSystem == NULL) {
        destructibleSystem = dynamic_cast<DestructibleSystem*>(EntityManager::instance->systems[SystemIDs::DestructibleID]);
    }



    //TODO supprimer
    // RigidBodySystem* rbSystem = dynamic_cast<RigidBodySystem*>(EntityManager::instance->systems[SystemIDs::RigidBodyID]);
    // rbSystem->setGravity(glm::vec3(0.0f, -1.0f, 0.0f));
}

void CrousManControllerSystem::update(unsigned short i, unsigned short entityID) {
    if(DEBUG_CONTROLLER) std::cout << "UPDATE ON CROUSSMAN CONTROLLER" << std::endl;
    
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
    glm::vec3 velocityNoY = glm::normalize(glm::vec3(rb->velocity.x, 0.0, rb->velocity.z));

    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
        currentMaxSpeed *= 10.0f;
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
    
    // apply input

    rb->addAcceleration(appliedAcceleration);

    if (!mooved) {
        if (glm::dot(velocityNoY, velocityNoY) > FLT_EPSILON)
            rb->addAcceleration(-accelerationAmount * 2.0f * glm::normalize(velocityNoY));
        else
            rb->velocity = glm::vec3(0.0f, rb->velocity.y, 0.0f);
    }

    if (DEBUG_CONTROLLER) std::cout << "LASER RAYCAST " << std::endl;

    // Laser : raycast
    if (glfwGetMouseButton(InputManager::instance->window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS) {
        Ray* ray = new Ray(cameraWTr->translation, cameraWTr->getForward());
        std::vector<RaycastResult*> rayResult = colliderSystem->rayCastAll(*ray);
        RaycastResult* closest = NULL;
        float minDistance = FLT_MAX;
        float distance;
        glm::vec3 cameraToResult;

        if (DEBUG_CONTROLLER) std::cout << "1 " << std::flush;

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
        if (DEBUG_CONTROLLER) std::cout << "2 " << std::flush;

        glm::vec3 laserHit;
        if (closest != NULL) {
            if (DEBUG_CONTROLLER) std::cout << "IF " << std::flush;

            distance = sqrt(distance);
            laserHit = closest->point;

            std::cout << closest->entityIDCollid << std::flush;

            if (EntityManager::instance->hasComponent(SystemIDs::DestructibleID, closest->entityIDCollid)) {
                if (DEBUG_CONTROLLER) std::cout << "IM DESTRUCTIBLE " << std::flush;


                unsigned int destructibleID = EntityManager::instance->getComponentId(SystemIDs::DestructibleID, closest->entityIDCollid);
                if (DEBUG_CONTROLLER) std::cout << "ID : " << destructibleID << " " << std::flush;

                destructibleSystem->destroyAmount(destructibleID, 1.0f * InputManager::instance->deltaTime);


                if (closest != NULL && !crous->laserSoundCooldown->inCooldown()) {
                    crous->laserSoundCooldown->start();
                    SoundManager::instance->playAt("../ressources/Sounds/laserHit.wav", laserHit);
                }
            }
        }
        else {
            if (DEBUG_CONTROLLER) std::cout << "ELSE " << std::flush;

            distance = 200.0f;
            laserHit = cameraWTr->translation + distance * cameraWTr->getForward();
        }

        if (DEBUG_CONTROLLER) std::cout << "3 " << std::flush;

        float laserSize = sin(InputManager::instance->lastFrame * 15.0f) * 1.0f + 2.0f;
        glm::vec3 crousToHit = laserHit - tr->translation;
        crous->laserEntity->transform->translation = tr->translation + crousToHit * 0.5f;
        crous->laserEntity->transform->rotation.lookAt(crous->laserEntity->transform->translation, laserHit, glm::vec3(0.0, 1.0, 0.0));
        crous->laserEntity->transform->scaling = glm::vec3(laserSize, laserSize, distance * 0.5f);
        crous->laserEntity->setActiveRecursive(true);


        // Laser Sounds
        if (DEBUG_CONTROLLER) std::cout << "4 " << std::flush;

        if (crous->firstLaserSound) {
            SoundManager::instance->playOver("../ressources/Sounds/laserStart.wav", crousEntity);
            crous->firstLaserSound = false;
        }
        else if (crous->laserAudioInd == (unsigned int)-1) {
            crous->laserAudioInd = SoundManager::instance->playOver("../ressources/Sounds/laser.wav", crousEntity);
            SoundManager::instance->audios[crous->laserAudioInd].sound3D->setIsLooped(true);
        }
    }
    if (DEBUG_CONTROLLER) std::cout << "LASER RAYCAST END " << std::endl;

    if (glfwGetMouseButton(InputManager::instance->window, GLFW_MOUSE_BUTTON_1) == GLFW_RELEASE){
        crous->laserEntity->setActiveRecursive(false);
        if (crous->laserAudioInd != (unsigned int)-1) {
            SoundManager::instance->audios[crous->laserAudioInd].sound3D->setIsLooped(false);
            SoundManager::instance->audios[crous->laserAudioInd].sound3D->stop();
            crous->laserAudioInd = (unsigned int)-1;
            crous->firstLaserSound = true;
        }
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


        glm::vec3 diferenceVector = (tr->translation + crous->initialRotatingPos) - rotatingTr->translation;
        rotatingTr->translation += diferenceVector * 0.1f;
        rotatingTr->rotation.setRotation(crous->azimuth, glm::vec3(0.0, 1.0, 0.0));
        rotatingTr->rotation.combineRotation(-crous->zenith, glm::vec3(1.0, 0.0, 0.0));
        
        diferenceVector = tr->translation + glm::vec3(0.0, 7.0, 0.0) - saucisseEntityTr->translation;
        saucisseEntityTr->translation += diferenceVector * 0.2f;

        
        if (glm::dot(velocityNoY, velocityNoY) > 1.0f) {
            glm::vec3 target = tr->translation + velocityNoY;
            tr->rotation.lookAt(tr->translation, target, glm::vec3(0.0f, 1.0f, 0.0f));
            saucisseEntityTr->rotation.lookAt(tr->translation, target, glm::vec3(0.0f, 1.0f, 0.0f));
        }

        // Raycast between rotating and the camera to move it in front of the nearest wall

        glm::vec3 rotatingToCameraTarget = glm::normalize(cameraTargetWTr->translation - wtr->translation);
        Ray* ray = new Ray(wtr->translation, rotatingToCameraTarget);
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
        cameraTr->translation = wtr->translation + rotatingToCameraTarget * distance;
        glm::vec3 rotatingToCameraTarget2 = glm::normalize(cameraTargetWTr->translation - rotatingWTr->translation);
        cameraTr->lookAtDirection(rotatingToCameraTarget2);
    }

    if (DEBUG_CONTROLLER) std::cout << "UPDATE ON CROUSSMAN CONTROLLER END " << std::endl;
}

void CrousManControllerSystem::addEntityComponent() {
    EntityManager::instance->crousManControllerComponents.push_back(CrousManController());
}

CrousManController* CrousManControllerSystem::getCrousManController(unsigned short i) {
    return &EntityManager::instance->crousManControllerComponents[i];
}


