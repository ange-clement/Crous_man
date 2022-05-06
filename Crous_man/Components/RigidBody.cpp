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



//Particle RB type functions behavior
void RigidBodySystem::applyForcesParticlesRB(RigidBody* rb) {
    rb->forces = gravity;
}

glm::vec3 RigidBodySystem::updateParticlesRB_EulerIntegration(RigidBody* rb,const glm::vec3& currentPos, float deltaTime) {
    
    glm::vec3 newPos;
    
    rb->oldPosition = currentPos;
    glm::vec3 acceleration = rb->forces * (1.0f / rb->mass);
    rb->speed = rb->speed * friction + acceleration * deltaTime;
    
    newPos = currentPos + rb->speed * deltaTime;
    return newPos;
}

glm::vec3 RigidBodySystem::updateParticlesRB_VerletIntegration(RigidBody* rb, const glm::vec3& currentPos, float deltaTime) {
    glm::vec3 newPos;

    rb->oldPosition = currentPos;
    glm::vec3 acceleration = rb->forces * (1.0f / rb->mass);

    glm::vec3 oldVelocity = rb->speed;
    rb->speed = rb->speed * friction + acceleration * deltaTime;
    newPos = currentPos + (oldVelocity + rb->speed) * 0.5f * deltaTime;
    return newPos;
}


glm::vec3 resolveConstraintParticles(ColliderResult* res, RigidBody* rb, const glm::vec3& currentPos) {
    return glm::vec3(0);
}