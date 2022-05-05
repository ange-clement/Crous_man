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

#include "RigidBody.hpp"

RigidBodySystem::RigidBodySystem() : ComponentSystem() {
    requiredComponentsBitmap = new Bitmap({ SystemIDs::RigidBodyID });
}

RigidBodySystem::~RigidBodySystem() {

}

void RigidBodySystem::initialize(unsigned short i, unsigned short entityID) {

}

void RigidBodySystem::updateOnCollide(unsigned short i, unsigned short entityID, const std::vector<ColliderResult*>& collisionResults) {
    //TODO appliquer les forces de collision rigides
}

void RigidBodySystem::updatePhysics(unsigned short i, unsigned short entityID) {
    RigidBody* rb = getRigidBody(i);
    float deltaTime = InputManager::instance->deltaTime;
    rb->speed += (rb->acceleration + this->gravity*0.0f) * deltaTime;
    Entity* e = EntityManager::instance->entities[entityID];

    e->transform->translate(rb->speed);
    e->worldTransform->translate(rb->speed);
}

void RigidBodySystem::addEntityComponent() {
    EntityManager::instance->rigidBodyComponents.push_back(RigidBody());
}

RigidBody* RigidBodySystem::getRigidBody(unsigned short i) {
    return &EntityManager::instance->rigidBodyComponents[i];
}