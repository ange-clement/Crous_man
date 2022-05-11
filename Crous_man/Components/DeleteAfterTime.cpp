#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "../InputManager.hpp"
#include "../ECS/EntityManager.hpp"
#include "../ECS/Bitmap.hpp"
#include "../ECS/Entity.hpp"

#include "../Transform.hpp"

#include "DeleteAfterTime.hpp"

DeleteAfterTimeSystem::DeleteAfterTimeSystem() : ComponentSystem(){
    requiredComponentsBitmap = new Bitmap({SystemIDs::DeleteAfterTimeID});
}

DeleteAfterTimeSystem::~DeleteAfterTimeSystem() {

}

void DeleteAfterTimeSystem::initialize(unsigned short i, unsigned short entityID) {

}

void DeleteAfterTimeSystem::update(unsigned short i, unsigned short entityID) {
    Entity* e = EntityManager::instance->entities[entityID];
    DeleteAfterTime* dat = getDeleteAfterTime(i);
    dat->timeLeftToDelete -= InputManager::instance->deltaTime;
    dat->timeBeforeScaling -= InputManager::instance->deltaTime;
    if (dat->timeBeforeScaling <= 0 && dat->timeLeftToDelete > 0) {
        if (dat->invTimeDiff < 0) {
            dat->invTimeDiff = 1.0 / (dat->timeLeftToDelete - dat->timeBeforeScaling);
            dat->originalScale = e->transform->scaling;
        }
        float scaleAmount = dat->timeLeftToDelete * dat->invTimeDiff;
        e->transform->scaling = dat->originalScale * scaleAmount;
    }
    if (dat->timeLeftToDelete < 0) {
        EntityManager::instance->removeEntity(e);
    }
}

void DeleteAfterTimeSystem::addEntityComponent() {
    EntityManager::instance->deleteAfterTimeComponents.push_back(DeleteAfterTime());
}

DeleteAfterTime* DeleteAfterTimeSystem::getDeleteAfterTime(unsigned short i) {
    return &EntityManager::instance->deleteAfterTimeComponents[i];
}