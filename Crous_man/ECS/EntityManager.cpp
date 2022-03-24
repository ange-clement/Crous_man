#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "SystemIDs.hpp"
#include "Bitmap.hpp"

#include "EntityManager.hpp"

#include "../Components/Spin.hpp"

#include "ComponentSystem.hpp"
#include "Entity.hpp"
#include "../Transform.hpp"

EntityManager* EntityManager::instance = NULL;

EntityManager::EntityManager() {
    if (EntityManager::instance == NULL) {
        EntityManager::instance = this;
        Entity* root = new Entity();
        root->id = 0;
        entities.push_back(root);
        initSystems();
    } else {
        std::cerr << "Error : cannot instanciate two EntityManager" << std::endl;
    }
}

EntityManager::~EntityManager() {
    entities.clear();
    spinComponents.clear();
    EntityManager::instance = NULL;
}

void EntityManager::initSystems() {
    systems.resize(SystemIDs::NUMBER);
    systems[SystemIDs::SpinID] = new SpinSystem();
}

void EntityManager::initializeAllSystems() {
    for (size_t i = 0, size = systems.size(); i < size; i++) {
        systems[i]->initializeAll();
    }
}

void EntityManager::updateAllSystems() {
    for (size_t i = 0, size = systems.size(); i < size; i++) {
        systems[i]->updateAll();
    }
}

void EntityManager::updateTransforms() {
    this->entities[0]->updateTransforms();
}


unsigned short EntityManager::getComponentId(SystemIDs system, unsigned short entityID) {
    return systems[system]->getComponentId(entityID);
}

bool EntityManager::hasComponent(SystemIDs system, unsigned short entityID) {
    Bitmap* b = this->systems[system]->requiredComponentsBitmap;
    return this->entities[entityID]->componentsBitmap->combine(b)->equals(b);
}

void EntityManager::addEntity(Entity* entity) {
    unsigned short id = entities.size();

    Bitmap* entityBitmap;
    Bitmap* systemBitmap;

    for (ComponentSystem* system : systems) {
        entityBitmap = entity->componentsBitmap;
        systemBitmap = system->requiredComponentsBitmap;
        if (entityBitmap->combine(systemBitmap)->equals(systemBitmap)) {
            system->addEntity(id);
        }
    }

    entity->id = id;
    entities.push_back(entity);
    entities[0]->addChildren(entity);
}