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
#include "Entity.hpp"

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

Entity* getEntity(unsigned short entityID) {
    return EntityManager::instance->entities[entityID];
}


void ComponentSystem::update(unsigned short i, unsigned short entityID) {}
void ComponentSystem::addEntityComponent() {}