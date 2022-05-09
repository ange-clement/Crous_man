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

#include "../Transform.hpp"

#include "FollowObject.hpp"
#include "../ECS/Entity.hpp"

FollowObjectSystem::FollowObjectSystem() : ComponentSystem() {
    requiredComponentsBitmap = new Bitmap({ SystemIDs::FollowObjectID });
}

FollowObjectSystem::~FollowObjectSystem() {

}

void FollowObjectSystem::initialize(unsigned short i, unsigned short entityID) {

}

void FollowObjectSystem::updateAfterPhysics(unsigned short i, unsigned short entityID) {
    FollowObject* f = getFollowObject(i);
    if (f->entityToFollow != NULL) {
        Entity* e = EntityManager::instance->entities[entityID];
        glm::vec3 diferenceVector = f->entityToFollow->worldTransform->translation - e->transform->translation;
        e->transform->translation += diferenceVector * f->interpolationAmount;
        e->transform->lookAtDirection( - f->entityToFollow->worldTransform->getForward());
    }
}

void FollowObjectSystem::addEntityComponent() {
    EntityManager::instance->followObjectComponents.push_back(FollowObject());
}

FollowObject* FollowObjectSystem::getFollowObject(unsigned short i) {
    return &EntityManager::instance->followObjectComponents[i];
}