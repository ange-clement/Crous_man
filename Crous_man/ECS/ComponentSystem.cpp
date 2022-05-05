#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "EntityManager.hpp"

#include "../Components/Collider.hpp"

#include "ComponentSystem.hpp"

#include "Bitmap.hpp"

ComponentSystem::ComponentSystem() {
    colliderSystemInstance = NULL;
}

ComponentSystem::~ComponentSystem() {

}


void ComponentSystem::updateAll() {
    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        if (entityIDs[i] == (unsigned short)-1)
            continue;
        this->update(i, entityIDs[i]);
    }
}
void ComponentSystem::updateCollisionAll() {
    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        if (entityIDs[i] == (unsigned short)-1)
            continue;
        this->updateCollision(i, entityIDs[i]);
    }
}
void ComponentSystem::updateOnCollideAll() {
    if (colliderSystemInstance == NULL)
        colliderSystemInstance = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID]);
    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        if (entityIDs[i] == (unsigned short)-1)
            continue;
        if (!EntityManager::instance->hasComponent(SystemIDs::ColliderID, entityIDs[i]))
            continue;
        std::vector<ColliderResult*> collisionResults;
        std::vector<ColliderResult*> collisionMapResult = colliderSystemInstance->getResultOf(entityIDs[i]);
        for (unsigned int c = 0, size = collisionMapResult.size(); c < size; c++) {
            if (collisionMapResult[c] != NULL && collisionMapResult[c]->isInCollision) {
                collisionResults.push_back(collisionMapResult[c]);
            }
        }
        if (collisionResults.size() == 0)
            continue;
        this->updateOnCollide(i, entityIDs[i], collisionResults);
    }
}
void ComponentSystem::updatePhysicsAll() {
    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        if (entityIDs[i] == (unsigned short) -1)
            continue;
        this->updatePhysics(i, entityIDs[i]);
    }
}
void ComponentSystem::updateAfterPhysicsAll() {
    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        if (entityIDs[i] == (unsigned short) -1)
            continue;
        this->updateAfterPhysics(i, entityIDs[i]);
    }
}
void ComponentSystem::renderAll() {
    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        if (entityIDs[i] == (unsigned short) -1)
            continue;
        this->render(i, entityIDs[i]);
    }
}
void ComponentSystem::updateAfterRenderAll() {
    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        if (entityIDs[i] == (unsigned short) -1)
            continue;
        this->updateAfterRender(i, entityIDs[i]);
    }
}



void ComponentSystem::update(unsigned short i, unsigned short entityID) {

}
void ComponentSystem::updateCollision(unsigned short i, unsigned short entityID) {

}
void ComponentSystem::updateOnCollide(unsigned short i, unsigned short entityID, const std::vector<ColliderResult*> & collisionResults) {

}
void ComponentSystem::updatePhysics(unsigned short i, unsigned short entityID) {

}
void ComponentSystem::updateAfterPhysics(unsigned short i, unsigned short entityID) {

}
void ComponentSystem::render(unsigned short i, unsigned short entityID) {

}
void ComponentSystem::updateAfterRender(unsigned short i, unsigned short entityID) {

}


void ComponentSystem::initializeAll() {
    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        if (entityIDs[i] == (unsigned short)-1)
            continue;
        this->initialize(i, entityIDs[i]);
    }
}

void ComponentSystem::addEntity(unsigned short entityID) {
    this->entityIDs.push_back(entityID);
    this->addEntityComponent();
}

void ComponentSystem::removeEntity(unsigned short entityID) {
    //Just mark it as re-writable
    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        if (entityIDs[i] == entityID) {
            this->entityIDs[i] = (unsigned short) -1;
            return;
        }
    }
}

bool ComponentSystem::containsEntity(unsigned short entityID) {
    return getComponentId(entityID) != (unsigned short)-1;
}

unsigned short ComponentSystem::getComponentId(unsigned short entityID) {
    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        if (entityIDs[i] == entityID) {
            return i;
        }
    }
    return (unsigned short) -1;
}


void ComponentSystem::addEntityComponent() {}