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
#include <Crous_man/SoundManager.hpp>

#include "../Collider.hpp"
#include "../RigidBody.hpp"

#include "SimpleMovementPlayer.hpp"

void SimpleMovementPlayer::applyRbMovement(glm::vec3 amount) {
    glm::vec3 posAt = glm::vec3(0.5f, 0.0f, 0.0f);
    switch (this->rbMovementType)
    {
    case 0:
        rb->addForce(amount);
        break;
    case 1:
        rb->addAcceleration(amount);
        break;
    case 2:
        rb->addVelocity(amount);
        break;
    case 3:
        rb->addImpulse(amount);
        break;
    case 4:
        rb->addAngularForce(amount);
        break;
    case 5:
        rb->addAngularAcceleration(amount);
        break;
    case 6:
        rb->addAngularVelocity(amount);
        break;
    case 7:
        rb->addAngularImpulse(amount);
        break;
    case 8:
        rb->addForceAtPosition(amount, posAt);
        break;
    case 9:
        rb->addAccelerationAtPosition(amount, posAt);
        break;
    case 10:
        rb->addVelocityAtPosition(amount, posAt);
        break;
    case 11:
        rb->addImpulseAtPosition(amount, posAt);
        break;
    default:
        break;
    }
}

SimpleMovementPlayerSystem::SimpleMovementPlayerSystem() : ComponentSystem() {
    requiredComponentsBitmap = new Bitmap({ SystemIDs::SimplePlayerControllerID });
}

SimpleMovementPlayerSystem::~SimpleMovementPlayerSystem() {}



void SimpleMovementPlayerSystem::initialize(unsigned short i, unsigned short entityID) {
    SimpleMovementPlayer* fc = getSimpleMovementController(i);
    fc->speed = 40.0f;
    if (rbSystem == NULL) {
        rbSystem = dynamic_cast<RigidBodySystem*>(EntityManager::instance->systems[SystemIDs::RigidBodyID]);
        //rbSystem->setGravity(glm::vec3(0.0f));
    }

    if (EntityManager::instance->hasComponent(SystemIDs::RigidBodyID, entityID)) {
        fc->rb = rbSystem->getRigidBody(EntityManager::instance->getComponentId(SystemIDs::RigidBodyID, entityID));
    }
}

void SimpleMovementPlayerSystem::update(unsigned short i, unsigned short entityID) {
    Entity* e = EntityManager::instance->entities[entityID];
    SimpleMovementPlayer* sp = getSimpleMovementController(i);
    Transform* tr = EntityManager::instance->entities[entityID]->transform;

    float translationAmount = sp->speed * InputManager::instance->deltaTime;

    glm::vec3 rdirection = glm::vec3(1, 0, 0);
    glm::vec3 udirection = glm::vec3(0.0, 1.0, 0.0);
    glm::vec3 fdirection = glm::vec3(0, 0.0, 1);

    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
        translationAmount *= 4;
    }
    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS) {
        translationAmount /= 32;
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

    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_K) == GLFW_PRESS) {
        tr->rotation.combineRotation(.01, glm::vec3(1, 0, 0));
    }
    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_L) == GLFW_PRESS) {
        tr->rotation.combineRotation(.01, glm::vec3(0, 1, 0));
    }


    if (sp->rb != NULL) {
        float forceAmount = 1.0f;

        if (!sp->movedRbMovementType && glfwGetKey(InputManager::instance->window, GLFW_KEY_Z) == GLFW_PRESS) {
            sp->movedRbMovementType = true;
            sp->rbMovementType++;
            sp->rbMovementType %= 12;
            std::cout << sp->rbMovementType << std::endl;

            SoundManager::instance->playOver("../ressources/Sounds/start.wav", e);
            //SoundManager::instance->audios.back().sound3D->setIsLooped(true);
        }

        if (glfwGetKey(InputManager::instance->window, GLFW_KEY_Z) == GLFW_RELEASE) {
            sp->movedRbMovementType = false;
        }



        if (glfwGetKey(InputManager::instance->window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
            forceAmount *= 4;
        }
        if (glfwGetKey(InputManager::instance->window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS) {
            forceAmount /= 8;
        }

        /*if (glfwGetKey(InputManager::instance->window, GLFW_KEY_X) == GLFW_PRESS) {
            rbSystem->setGravity(glm::vec3(0.0, -0.1f, 0.0) * forceAmount);
        }
        if (glfwGetKey(InputManager::instance->window, GLFW_KEY_X) == GLFW_RELEASE) {
            rbSystem->setGravity(glm::vec3(0.0, 0.0f, 0.0) * forceAmount);
        }*/

        if (glfwGetKey(InputManager::instance->window, GLFW_KEY_V) == GLFW_PRESS) {
            sp->rb->addForce(-glm::normalize(e->transform->translation) * forceAmount);
        }


        if (glfwGetKey(InputManager::instance->window, GLFW_KEY_T) == GLFW_PRESS) {
            sp->applyRbMovement(forceAmount * fdirection);
        }

        if (glfwGetKey(InputManager::instance->window, GLFW_KEY_G) == GLFW_PRESS) {
            sp->applyRbMovement(-forceAmount * fdirection);
        }

        if (glfwGetKey(InputManager::instance->window, GLFW_KEY_F) == GLFW_PRESS) {
            sp->applyRbMovement(forceAmount * rdirection);
        }

        if (glfwGetKey(InputManager::instance->window, GLFW_KEY_H) == GLFW_PRESS) {
            sp->applyRbMovement(-forceAmount * rdirection);
        }

        if (glfwGetKey(InputManager::instance->window, GLFW_KEY_Y) == GLFW_PRESS) {
            sp->applyRbMovement(forceAmount * udirection);
        }
        if (glfwGetKey(InputManager::instance->window, GLFW_KEY_R) == GLFW_PRESS) {
            sp->applyRbMovement(-forceAmount * udirection);
        }
    }
}


void SimpleMovementPlayerSystem::updateOnCollide(unsigned short i, unsigned short entityID, const std::vector<ColliderResult*>& collisionResults) {
    
    //std::cout << "COLISION with " << collisionResults.size() << " elements : " << std::endl;
    //for (unsigned int c = 0, size = collisionResults.size(); c < size; c++) {
    //    std::cout << c << std::endl;
    //    std::cout << collisionResults[c]->isInCollision << std::endl;
    //    std::cout << collisionResults[c]->entityCollidID << std::endl;
    //    //std::cout << collisionResults[c]->penetrationDistance << std::endl;
    //    //std::cout << collisionResults[c]->pointCollision[0] << " " << collisionResults[c]->pointCollision[1] << " " << collisionResults[c]->pointCollision[2] << std::endl;
    //}
}

void SimpleMovementPlayerSystem::addEntityComponent() {
    EntityManager::instance->simplePlayerControllerComponents.push_back(SimpleMovementPlayer());
}

SimpleMovementPlayer* SimpleMovementPlayerSystem::getSimpleMovementController(unsigned short i) {
    return &EntityManager::instance->simplePlayerControllerComponents[i];
}