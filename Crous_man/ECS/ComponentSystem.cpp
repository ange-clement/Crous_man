#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "EntityManager.hpp"

#include "ComponentSystem.hpp"

#include "Bitmap.hpp"

ComponentSystem::ComponentSystem() {
    
}

ComponentSystem::~ComponentSystem() {

}


void ComponentSystem::updateAll() {
    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        this->update(i, entityIDs[i]);
    }
}
void ComponentSystem::updatePhysicsAll() {
    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        this->updatePhysics(i, entityIDs[i]);
    }
}
void ComponentSystem::updateAfterPhysicsAll() {
    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        this->updateAfterPhysics(i, entityIDs[i]);
    }
}
void ComponentSystem::renderAll() {
    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        this->render(i, entityIDs[i]);
    }
}
void ComponentSystem::updateAfterRenderAll() {
    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        this->updateAfterRender(i, entityIDs[i]);
    }
}



void ComponentSystem::update(unsigned short i, unsigned short entityID) {

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
        this->initialize(i, entityIDs[i]);
    }
}

void ComponentSystem::addEntity(unsigned short entityID) {
    this->entityIDs.push_back(entityID);
    this->addEntityComponent();
}

void ComponentSystem::removeEntity(unsigned short entityID) {
    //TODO
    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        if (entityIDs[i] == entityID) {
            this->entityIDs[i] = (unsigned short) -1;
        }
    }
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