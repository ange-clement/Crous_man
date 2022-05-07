#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "../Util.hpp"

#include "../InputManager.hpp"
#include "../ECS/EntityManager.hpp"
#include "../ECS/Bitmap.hpp"
#include "../ECS/Entity.hpp"

#include "Collider.hpp"
#include <common/ray.hpp>

#include "../Transform.hpp"

#include "RigidBody.hpp"

void RigidBody::setMass(float mass) {
    if (mass == 0) {
        mass = FLT_EPSILON;
    }
    this->mass = mass;
    this->inverseOfMass = 1.0 / mass;
}

void RigidBody::addForce(glm::vec3 force) {
    this->combinedForces += force;
}

void RigidBody::addAcceleration(glm::vec3 acc) {
    this->combinedForces += acc * this->mass;
}

void RigidBody::addVelocity(glm::vec3 vel) {
    this->velocity += vel;
}

void RigidBody::addImpulse(glm::vec3 impulse) {
    this->velocity += impulse * this->inverseOfMass;
}

void RigidBody::addAngularForce(glm::vec3 force) {
    this->combinedAngularForces += force;
}

void RigidBody::addAngularAcceleration(glm::vec3 acc) {
    this->combinedAngularForces += acc * this->mass;
}

void RigidBody::addAngularVelocity(glm::vec3 vel) {
    this->angularSpeed += vel;
}

void RigidBody::addAngularImpulse(glm::vec3 impulse) {
    this->angularSpeed += impulse * this->inverseOfMass;
}

void RigidBody::addForceAtPosition(glm::vec3 force, glm::vec3 pos) {
    glm::vec3 centerOfMass = glm::vec3(0.0f);

    glm::vec3 centerToPosVector = pos - centerOfMass;
    float posDistance = glm::length(centerToPosVector);

    float forceLength = glm::length(force);
    /*
    float forceStrength = forceLength * posDistance;
    float forceAngle = forceStrength / posDistance;
    */
    float forceAngle = forceLength;

    //Rotation axis vector
    glm::vec3 forceAxis = glm::cross(force, centerToPosVector);
    
    //Apply a force 
    float forceAxisLength = glm::length(forceAxis);
    //If vector are non aligned
    if (forceAxisLength > 0.0f) {
        forceAxis = forceAxis / forceAxisLength;
        //We add angular force
        this->addAngularForce(forceAxis * forceAngle);
    }
    //By default, we add normal force
    this->addForce(force / ((posDistance < 1.0) ? 1.0f : posDistance));
}

void RigidBody::addAccelerationAtPosition(glm::vec3 acc, glm::vec3 pos) {
    this->addForceAtPosition(acc * this->mass, pos);
}


//Velocity mechanism
void RigidBody::addVelocityAtPosition(glm::vec3 vel, glm::vec3 pos) {
    glm::vec3 centerOfMass = glm::vec3(0.0f);

    glm::vec3 centerToPosVector = pos - centerOfMass;
    float posDistance = glm::length(centerToPosVector);

    float velLength = glm::length(vel);
    float velAngle = velLength;

    glm::vec3 velAxis = glm::normalize(glm::cross(vel, centerToPosVector));
    float velAxisLength = glm::length(velAxis);
    if (velAxisLength > 0.0f) {
        velAxis = velAxis / velAxisLength;
        this->addAngularVelocity(velAxis * velAngle);
    }
    this->addVelocity(vel / ((posDistance < 1.0) ? 1.0f : posDistance));
}

void RigidBody::addImpulseAtPosition(glm::vec3 impulse, glm::vec3 pos) {
    this->addVelocityAtPosition(impulse * this->inverseOfMass, pos);
}





RigidBodySystem::RigidBodySystem() : ComponentSystem() {
    requiredComponentsBitmap = new Bitmap({ SystemIDs::RigidBodyID });
}

RigidBodySystem::~RigidBodySystem() {

}

void RigidBodySystem::setGravity(glm::vec3 gravity) {
    this->gravity = gravity;
    if (glm::length(gravity) > 0) {
        this->gravityDirection = glm::normalize(gravity);
    }
}

void RigidBodySystem::initialize(unsigned short i, unsigned short entityID) {

}

void RigidBodySystem::updateOnCollide(unsigned short i, unsigned short entityID, const std::vector<ColliderResult*>& collisionResults) {
    //TODO appliquer les forces de collision rigides
    RigidBody* rb = getRigidBody(i);
    RigidBody* otherRb;

    for (unsigned int c = 0, size = collisionResults.size(); c < size; c++) {
        otherRb = getRigidBodyFromEntityId(collisionResults[c]->entityCollidID);
        
        if (otherRb == NULL) {
            continue;
        }


        // Friction calculation
        //      Maybe only compute if velocity toward colision is big enough
        float combinedStaticFriction  = 0.5 * (rb->staticFriction  + otherRb->staticFriction);
        float combinedCineticFriction = 0.5 * (rb->cineticFriction + otherRb->cineticFriction);

        glm::vec3 normal  = glm::vec3(0.0, 1.0, 0.0); // TODO
        glm::vec3 tangent = glm::cross(normal, glm::cross(normal, gravityDirection)); // A tester
        rb->combinedStaticFriction  += tangent * combinedStaticFriction;
        rb->combinedCineticFriction += tangent * combinedCineticFriction;

        // Colision resolution

    }
}

void RigidBodySystem::updatePhysics(unsigned short i, unsigned short entityID) {
    RigidBody* rb = getRigidBody(i);
    float deltaTime = InputManager::instance->deltaTime;

    float frictionLength = squareLength(rb->combinedStaticFriction);

    //If forces are to small, we simply dont MAJ velocity of the object
    if (frictionLength > 0.0f && glm::dot(rb->combinedStaticFriction, rb->combinedForces) / frictionLength < 1.0) {
        // forces inferior of friction forces : no mouvement
        rb->velocity = glm::vec3(0.0f);
    }
    else {
        glm::vec3 dragForces = rb->velocity * -rb->drag;
        glm::vec3 totalForces = rb->combinedForces + rb->combinedCineticFriction + dragForces;
        glm::vec3 acceleration = totalForces * rb->inverseOfMass;

        glm::vec3 angularDragForces = rb->angularSpeed* -rb->angularDrag;
        glm::vec3 totalAngularForces = rb->combinedAngularForces + angularDragForces;
        glm::vec3 angularAcceleration = totalAngularForces * rb->inverseOfMass;

        rb->velocity += acceleration * deltaTime;
        rb->angularSpeed+= angularAcceleration * deltaTime;
        Entity* e = EntityManager::instance->entities[entityID];

        e->transform->translate(rb->velocity);
        e->worldTransform->translate(rb->velocity);

        float rotationAngle = glm::length(rb->angularSpeed);
        if (rotationAngle != 0.0f) {
            e->transform->rotation.combineRotation(rotationAngle, rb->angularSpeed);
        }
    }

    // Reinitialise forces (free fall object)
    rb->combinedForces = gravity * rb->mass;        // initialised by weight
    rb->combinedAngularForces   = glm::vec3(0.0f);
    rb->combinedStaticFriction  = glm::vec3(0.0f);
    rb->combinedCineticFriction = glm::vec3(0.0f);
}



void RigidBodySystem::addEntityComponent() {
    EntityManager::instance->rigidBodyComponents.push_back(RigidBody());
}

RigidBody* RigidBodySystem::getRigidBody(unsigned short i) {
    return &EntityManager::instance->rigidBodyComponents[i];
}

RigidBody* RigidBodySystem::getRigidBodyFromEntityId(unsigned short entityID) {
    if (!EntityManager::instance->hasComponent(SystemIDs::RigidBodyID, entityID)) {
        // If entity doesn't have a RigidBody, return NULL
        return NULL;
    }
    
    if (entityIDsToIndex.size() < entityID) {
        // If entityIDsToIndex is too small, extend it
        entityIDsToIndex.resize(entityID+1, (unsigned short)-1);
    }

    unsigned short id = entityIDsToIndex[entityID];
    if (id == (unsigned short)-1) {
        // If first time we called getRigidBodyFromEntityId with this entity, get it's ID and update the list
        id = getComponentId(entityID);
        entityIDsToIndex[entityID] = id;
    }

    if (id == (unsigned short)-1) {
        // If we could get it's ID, return NULL
        return NULL;
    }
    return getRigidBody(id);
}




void RigidBodySystem::initForcesParticlesRB(RigidBody* rb) {
    rb->combinedForces              = gravity * rb->mass;
    rb->combinedAngularForces       = glm::vec3(0.0f);
    rb->combinedStaticFriction      = glm::vec3(0.0f);
    rb->combinedCineticFriction     = glm::vec3(0.0f);
}

glm::vec3 RigidBodySystem::updateParticlesRB_EulerIntegration(RigidBody* rb, const glm::vec3& currentPos, float deltaTime) {
    rb->oldPosition = currentPos;
    glm::vec3 acceleration = rb->combinedForces * rb->inverseOfMass;
    rb->velocity = rb->velocity * rb->cineticFriction + acceleration * deltaTime;
    return currentPos + rb->velocity * deltaTime;
}

glm::vec3 RigidBodySystem::updateParticlesRB_AccurateEulerIntegration(RigidBody* rb, const glm::vec3& currentPos, float deltaTime) {
    rb->oldPosition = currentPos;
    glm::vec3 acceleration = rb->combinedForces * rb->inverseOfMass;
    glm::vec3 oldVelocity = rb->velocity;
    rb->velocity = rb->velocity * rb->cineticFriction + acceleration * deltaTime;
    return currentPos + (oldVelocity + rb->velocity) * 0.5f * deltaTime;
}


glm::vec3 RigidBodySystem::updateParticlesRB_VerletIntegration(RigidBody* rb, const glm::vec3& currentPos, float deltaTime) {
    //Find the implicit velocity of the particle
    glm::vec3 velocity = currentPos - rb->oldPosition;
    rb->oldPosition = currentPos;
    float deltaSquare = deltaTime * deltaTime;
    return currentPos +(velocity * rb->cineticFriction + rb->combinedForces * deltaSquare);
}

glm::vec3 RigidBodySystem::resolveConstraintParticles_Verlet(Collider& collider, RigidBody* rb_particles, const glm::vec3& currentPos) {
    glm::vec3 position = currentPos;
    if (LinetestCollider(collider, rb_particles->oldPosition, currentPos)) {
        glm::vec3 velocity = position - rb_particles->oldPosition;
        glm::vec3 direction = glm::normalize(velocity);

        Ray ray = Ray(rb_particles->oldPosition, direction);
        RaycastResult result;

        if (RayCastCollider(collider, ray, &result)) {
            // Place object just a little above collision result
            position = result.point + result.normal * 0.003f;

            //velocity vector into parallel and perpendicular components relative to the collision normal :
            glm::vec3 vn = result.normal * glm::dot(result.normal, velocity);
            glm::vec3 vt = velocity - vn;

            rb_particles->oldPosition = position - ((vt - vn) * rb_particles->bounce);
        }
    }
    return position;
}


glm::vec3 RigidBodySystem::resolveConstraintParticles_Euler(Collider& collider, RigidBody* rb_particles, const glm::vec3& currentPos) {
    glm::vec3 position = currentPos;
    if (LinetestCollider(collider, rb_particles->oldPosition, currentPos)) {
        glm::vec3 direction = glm::normalize(rb_particles->velocity);
        Ray ray = Ray(rb_particles->oldPosition, direction);
        RaycastResult result;

        if (RayCastCollider(collider, ray, &result)) {
            // Place object just a little above collision result
            position = result.point + result.normal * 0.003f;

            //velocity vector into parallel and perpendicular components relative to the collision normal :
            glm::vec3 vn = result.normal * glm::dot(result.normal, rb_particles->velocity);
            glm::vec3 vt = rb_particles->velocity - vn;

            rb_particles->oldPosition = position;
            rb_particles->velocity = vt - vn * rb_particles->bounce;
        }
    }
    return position;
}

