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


#include <Crous_man/ECS/EntityBuilder.hpp>

#include "../Transform.hpp"

#include "RigidBody.hpp"

#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <common/quaternion_utils.hpp>

#define DEBUG_RIGIDBODY false


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
    //this->velocity += impulse;
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
    //float forceAngle = forceLength;
    //float forceAngle = acos(forceLength / posDistance);
    //float forceAngle = forceLength / posDistance;

    //Rotation axis vector
    glm::vec3 forceAxis = glm::cross(force, centerToPosVector); // force axis = couple = F * r = m * r^2 * a --> il fautdiviser par r pour avoir la force!
    
    //Apply a force 
    float forceAxisLength = glm::length(forceAxis);
    //If vector are non aligned
    if (forceAxisLength > 0.0f) {
        //forceAxis = forceAxis / forceAxisLength;
        //We add angular force
        //this->addAngularForce(forceAxis * forceAngle);
        this->addAngularForce(forceAxis / posDistance);
    }
    this->addForce(force / (posDistance + 1.0f));
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
    float velAngle = acos(velLength / posDistance);

    glm::vec3 velAxis = glm::normalize(glm::cross(vel, centerToPosVector));
    float velAxisLength = glm::length(velAxis);
    if (velAxisLength > 0.0f) {
        velAxis = velAxis / velAxisLength;
        this->addAngularVelocity(velAxis * velAngle);
    }
    this->addVelocity(vel / (posDistance + 1.0f));
}

void RigidBody::addImpulseAtPosition(glm::vec3 impulse, glm::vec3 pos) {
    glm::vec3 centerOfMass = glm::vec3(0.0f);
    glm::vec3 centerToPosVector = pos - centerOfMass;


    float Iinv = 1.0f / (this->mass * glm::dot(centerToPosVector, centerToPosVector));
    glm::vec3 angularVelocityDiff = Iinv * glm::cross(centerToPosVector, impulse);

    glm::vec3 velocityDiff = impulse * this->inverseOfMass;


    this->addAngularVelocity(angularVelocityDiff);
    this->addVelocity(velocityDiff);
}


void printRB(RigidBody* rb) {
    std::cout << "VELOCITY : ";
    print(rb->velocity);
    std::cout << "ANGULAR SPEED : ";
    print(rb->angularSpeed);
    std::cout << "MASS : " << rb->mass << std::endl;
    std::cout << "inverse MASS : " << rb->inverseOfMass << std::endl;

    std::cout << "static friction : " << rb->mass << std::endl;
    std::cout << "cinetic friction : " << rb->inverseOfMass << std::endl;
    std::cout << "COMBINED FORCES : ";
    print(rb->combinedForces);

    std::cout << "DRAG : " << rb->drag << std::endl;
    std::cout << "ANGULAR DRAG : " << rb->angularDrag << std::endl;
}



RigidBodySystem::RigidBodySystem() : ComponentSystem() {
    requiredComponentsBitmap = new Bitmap({ SystemIDs::RigidBodyID });

    colliderSystem = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID]);
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
    // 1- Get all contacts
    // 2- Accumulate forces acting on the rigidbodies
    // 3- Apply impulses to resolve collisions (except if static)

    if (DEBUG_RIGIDBODY) {
        std::cout << "============== UPDATE ON COLID : " << i << std::endl;
        std::cout << "UPDATE ON COLID entityID : " << entityID << std::endl;
        std::cout << "COL RES SIZE : " << collisionResults.size() << std::endl;
    }

    /*=========================================================================================================================*/
    with_dynamic_friction &= with_friction;

    RigidBody* rb = getRigidBody(i);
    //std::cout << "RB : " << rb << std::endl;

    RigidBody* otherRb;

    Entity* e = EntityManager::instance->entities[entityID];
    Entity* otherE;

    Collider* col = getColliderFromEntityID(entityID);
    Collider* otherCol = 0;

    //std::cout << "PERFORM ANGULAR VELOCITY" << std::endl;
    //std::cout << "COLLIDER OF I : " << col << std::endl;


    //We can't perform angular impulse without Collider of the element 
    if (col != 0 || !with_rotation) {
        glm::mat4 inversedTensor_I;
        glm::mat4 inversedTensor_J;

        if (with_rotation) inversedTensor_I = getMatrixInverseTensor(entityID, *col, rb);

        glm::vec3 pos = e->worldTransform->translation;
        glm::vec3 otherpos;

        //std::cout << "FOR ON COLLISION RES" << std::endl;

        for (unsigned int c = 0, size = collisionResults.size(); c < size; c++) {
            if (DEBUG_RIGIDBODY) {
                std::cout << " ->COL WITH : " << collisionResults[c]->entityCollidID << std::endl;
            }

            //First get the entity and RB of the other member of the collision
            otherE = EntityManager::instance->entities[collisionResults[c]->entityCollidID];
            otherRb = getRigidBodyFromEntityId(collisionResults[c]->entityCollidID);

            if (otherRb == 0) {
                //std::cout << "COL WITH TRIGER : " << otherE << std::endl;
                continue;
            }

            if (correctPos) {
                //We can correct position of RB to avoid penetration
                glm::vec3 newpos;
                glm::vec3 otherNewpos;

                positionCorrection(newpos, otherNewpos, rb, otherRb, collisionResults[c]->contactsPts[0]);

                e->transform->translate(newpos);
                e->worldTransform->translate(newpos);

                otherE->transform->translate(otherNewpos);
                otherE->worldTransform->translate(otherNewpos);
            }


            otherpos = otherE->worldTransform->translation;

            if (with_rotation) {
                //We compute INVERSE TENSOR MATRIX
                //std::cout << "WITH ROTATION" << std::endl;
                otherCol = getColliderFromEntityID(collisionResults[c]->entityCollidID);
                if (otherCol != 0) {

                    inversedTensor_J = getMatrixInverseTensor(collisionResults[c]->entityCollidID, *otherCol, otherRb);
                    //std::cout << " ============================= I RECOMPUTED MATRIX colbis" << std::endl;
                    //print(inversedTensor_J);
                    //std::cout << " ============================= " << std::endl;
                    clearMatrixTensorStructure(collisionResults[c]->entityCollidID);

                }
                else {
                    std::cout << "NO COLLIDER, SKIPPED : " << collisionResults[c]->entityCollidID << std::endl;
                    continue;
                }
            }

            //Apply impulse for collision resolution
            unsigned int sizeContact = collisionResults[c]->contactsPts.size();
            if (sizeContact > 0) {
                //For each contacts points
                for (unsigned int p = 0; p < sizeContact; p++) {
                    ContactPoint* cp = collisionResults[c]->contactsPts[p];

                    rb->oldvelocity = rb->velocity;
                    otherRb->oldvelocity = otherRb->velocity;

                    if (with_rotation) {
                        if (with_friction) {
                            if (with_dynamic_friction) {
                                applyImpulse_linearAngularDynamicFriction(rb, otherRb, cp, sizeContact, pos, otherpos, inversedTensor_I, inversedTensor_J);
                            }
                            else {
                                applyImpulse_linearAngularFriction(rb, otherRb, cp, sizeContact, pos, otherpos, inversedTensor_I, inversedTensor_J);
                            }
                        }
                        else {
                            applyImpulse_linearAngular_bis(rb, otherRb, cp, sizeContact, pos, otherpos, inversedTensor_I, inversedTensor_J);
                            //applyImpulse_linearAngular(rb, otherRb, cp, sizeContact, pos, otherpos, inversedTensor_I, inversedTensor_J);
                        }
                    }
                    else {
                        if (with_friction) {
                            if (with_dynamic_friction) {
                                applyImpulse_linearDynamicFriction(rb, otherRb, cp, sizeContact);
                            }
                            else {
                                applyImpulse_linearFriction(rb, otherRb, cp, sizeContact);
                            }
                        }
                        else {
                            applyImpulse_linear(rb, otherRb, cp, sizeContact);
                        }
                    }
                }
            }
            else {
                std::cout << "trigger" << std::endl;
            }
        }
    }
    else {
        std::cout << "NO COLLIDER, SKIPPED : " << entityID << std::endl;
    }

}



void RigidBodySystem::updatePhysics(unsigned short i, unsigned short entityID) {
    //2 - Update position of every rb 
    //3 - Correct sinking using Linear Projection
    //4 - Solve contraint if applicable

    if (DEBUG_RIGIDBODY) {
        std::cout << "UPDATE ON PHYSICS RB i : " << i << std::endl;
        std::cout << "UPDATE ON PHYSICS RB id : " << entityID << std::endl;
    }

    /*=========================================================================================================================*/
    //if (InputManager::instance->nbFrames % frame_update_update_physics == 0) {
        RigidBody* rb = getRigidBody(i);
        //Apply basic gravity
        if (rb != 0 && !rb->static_RB) {

            float deltaTime = InputManager::instance->deltaTime;

            Entity* e = EntityManager::instance->entities[entityID];


            glm::vec3 currentPos = e->worldTransform->translation;


            //std::cout << "============ RB VALUE ============" << std::endl;
            //printRB(rb);

            //std::cout << "                     X VAL CUR P : " << currentPos.x << std::endl;
            //std::cout << "                     Y VAL CUR P : " << currentPos.y << std::endl;
            //std::cout << "                     Z VAL CUR P : " << currentPos.z << std::endl;

            
            //if (currentPos.x >= 0) {
            //    std::cout << "                     X VAL CUR P : " << currentPos.x << std::endl;
            //}
            //if (currentPos.y >= 0) {
            //    std::cout << "                     Y VAL CUR P : " << currentPos.y << std::endl;
            //}
            //if (currentPos.z >= 0) {
            //    std::cout << "=====================================" << std::endl;
            //    std::cout << "                     Z VAL CUR P : " << currentPos.z << std::endl;
            //}


            //linear update part
            glm::vec3 newPos = update_EulerIntegration(rb, currentPos, deltaTime);
            //glm::vec3 newPos = update_EulerIntegrationSemiImplicit(rb, currentPos, deltaTime);
            //glm::vec3 newPos = linear_update_AccurateEulerIntegration(rb, currentPos, deltaTime);
            //glm::vec3 newPos = linear_update_VerletIntegration(rb, currentPos, deltaTime);

            //Rotational update part
            glm::mat4 inversedTensor_I;

            Collider* col = getColliderFromEntityID(entityID);

            //std::cout << "GET COLLIDER FROM ENTITY ID" << std::endl;

            //We can't perform angular impulse without Collider of the element 
            if (with_rotation && col != 0) {

                //std::cout << "GET MATRICE INVERSE TENSOR" << std::endl;
                inversedTensor_I = getMatrixInverseTensor(entityID, *col, rb);
                //print(inversedTensor_I);

                rotational_update(rb, deltaTime, getFromMat4(inversedTensor_I));

                e->transform->rotation.combineRotation(degToRad(rb->angularSpeed.x), glm::vec3(1, 0, 0));
                e->transform->rotation.combineRotation(degToRad(rb->angularSpeed.y), glm::vec3(0, 1, 0));
                e->transform->rotation.combineRotation(degToRad(rb->angularSpeed.z), glm::vec3(0, 0, 1));

                //std::cout << "COMBINE ROT END" << std::endl;
            }

            // Apply on object
            glm::vec3 dif = newPos - currentPos;


            //std::cout << "UPDATE ON PHYSICS DIF" << std::endl;
            //print(dif);
            //print(newPos);

            e->transform->translate(dif);
            e->worldTransform->translate(dif);

            ApplyForces(rb);
        }
    //}
        //std::cout << "END UPDATE PHYS" << std::endl;
}

void RigidBodySystem::updateAfterPhysics(unsigned short i, unsigned short entityID) {
    //Preventing resized elements
    //if (InputManager::instance->nbFrames % frame_update_update_physics == 0) {
        RigidBody* rb = getRigidBody(i);
        if (rb != 0 && !rb->static_RB && with_rotation) {
            clearMatrixTensorStructure(entityID);
        }
    //}
}
glm::vec3 RigidBodySystem::update_EulerIntegration(RigidBody* rb, const glm::vec3& currentPos, float deltaTime) {
    float limit_speed = 0.001f;
    //std::cout << "EULER INTEGRATION" << std::endl;
    //std::cout << "DELTA TIME : " << deltaTime << std::endl;
    //print(rb->combinedForces * rb->inverseOfMass);
    //print(rb->velocity);

    //rb->oldPosition = currentPos;

    glm::vec3 acceleration = rb->combinedForces * rb->inverseOfMass;
    rb->velocity = rb->velocity * rb->drag;

    glm::vec3 velocity_applied = rb->velocity * deltaTime;
    rb->velocity = rb->velocity + (acceleration * deltaTime);

    //Nullify small velocities
    if (std::abs(velocity_applied.x) < limit_speed) {
        //std::cout << "RESET VELOCITY X" << std::endl;
        velocity_applied.x = 0.0f;
    }
    if (std::abs(velocity_applied.y) < limit_speed) {
        //std::cout << "RESET VELOCITY Y" << std::endl;
        velocity_applied.y = 0.0f;
    }
    if (std::abs(velocity_applied.z) < limit_speed) {
        //std::cout << "RESET VELOCITY Z" << std::endl;
        velocity_applied.z = 0.0f;
    }

    //std::cout << "VELOCITY APPLIED" << std::endl;
    //print(velocity_applied);

    return currentPos + velocity_applied;
    //std::cout << "=========== EULER INTEGRATION END ===========" << std::endl;
}
glm::vec3 RigidBodySystem::update_EulerIntegrationSemiImplicit(RigidBody* rb, const glm::vec3& currentPos, float deltaTime) {
    float limit_speed = 0.009f;
    //std::cout << "EULER INTEGRATION" << std::endl;
    //std::cout << "DELTA TIME : " << deltaTime << std::endl;
    //print(rb->combinedForces * rb->inverseOfMass);
    //print(rb->velocity);

    //rb->oldPosition = currentPos;

    glm::vec3 acceleration = rb->combinedForces * rb->inverseOfMass;
    rb->velocity = rb->velocity + (acceleration * deltaTime);
    rb->velocity = rb->velocity * rb->drag;

    glm::vec3 velocityapplied = rb->velocity * deltaTime;

    //Nullify small velocities
    if (std::abs(velocityapplied.x) < limit_speed) {
        //std::cout << "RESET VELOCITY X" << std::endl;
        velocityapplied.x = 0.0f;
    }
    if (std::abs(velocityapplied.y) < limit_speed) {
        //std::cout << "RESET VELOCITY Y" << std::endl;
        velocityapplied.y = 0.0f;
    }
    if (std::abs(velocityapplied.z) < limit_speed) {
        //std::cout << "RESET VELOCITY Z" << std::endl;
        velocityapplied.z = 0.0f;
    }

    //std::cout << "VELOCITY APPLIED" << std::endl;
    //print(velocityapplied);

    return currentPos + velocityapplied;
    //std::cout << "=========== EULER INTEGRATION END ===========" << std::endl;
}
glm::vec3 RigidBodySystem::update_AccurateEulerIntegration(RigidBody* rb, const glm::vec3& currentPos, float deltaTime) {
    rb->oldPosition = currentPos;
    glm::vec3 acceleration = rb->combinedForces * rb->inverseOfMass;
    glm::vec3 oldVelocity = rb->velocity;
    rb->velocity = rb->velocity * rb->drag + acceleration * deltaTime;

    //Nullify small velocities
    if (fabsf(rb->velocity.x) < 0.001f) {
        rb->velocity.x = 0.0f;
    }
    if (fabsf(rb->velocity.y) < 0.001f) {
        rb->velocity.y = 0.0f;
    }
    if (fabsf(rb->velocity.z) < 0.001f) {
        rb->velocity.z = 0.0f;
    }

    return currentPos + (oldVelocity + rb->velocity) * 0.5f * deltaTime;
}
glm::vec3 RigidBodySystem::update_VerletIntegration(RigidBody* rb, const glm::vec3& currentPos, float deltaTime) {
    //Find the implicit velocity of the particle
    glm::vec3 velocity = currentPos - rb->oldPosition;

    //Nullify small velocities
    if (fabsf(velocity.x) < 0.001f) {
        velocity.x = 0.0f;
    }
    if (fabsf(velocity.y) < 0.001f) {
        velocity.y = 0.0f;
    }
    if (fabsf(velocity.z) < 0.001f) {
        velocity.z = 0.0f;
    }

    rb->oldPosition = currentPos;
    float deltaSquare = deltaTime * deltaTime;
    return currentPos + (velocity * rb->drag + rb->combinedForces * deltaSquare);
}

void RigidBodySystem::rotational_update(RigidBody* rb, float deltaTime, const glm::mat3& inverseTensor) {

    if (DEBUG_RIGIDBODY) {
        std::cout << "ROTATIONNAL UPDATE" << std::endl;
    }
    glm::vec3 angAccel = rb->combinedAngularForces * inverseTensor;

    rb->angularSpeed = rb->angularSpeed + angAccel * deltaTime;
    rb->angularSpeed = rb->angularSpeed * rb->angularDrag;
    rb->angularSpeed *= deltaTime;


    //if the speed is to slow, no more rotation performed
    if (fabsf(rb->angularSpeed.x) < 0.001f) {
        if (DEBUG_RIGIDBODY) {
            std::cout << "X DEL" << std::endl;
        }
        rb->angularSpeed.x = 0.0f;
    }
    if (fabsf(rb->angularSpeed.y) < 0.001f) {
        if (DEBUG_RIGIDBODY) {
            std::cout << "Y DEL" << std::endl;
        }
        rb->angularSpeed.y = 0.0f;
    }
    if (fabsf(rb->angularSpeed.z) < 0.001f) {
        if (DEBUG_RIGIDBODY) {
            std::cout << "Z DEL" << std::endl;
        }
        rb->angularSpeed.z = 0.0f;
    }



    //  how to use it next :
    //  Entity* e = EntityManager::instance->entities[entityID];;
    //  e->transform->rotation.combineRotation(degToRad(rb->angularSpeed.x), glm::vec3(1,0,0));
    //  e->transform->rotation.combineRotation(degToRad(rb->angularSpeed.y), glm::vec3(0,1,0));
    //  e->transform->rotation.combineRotation(degToRad(rb->angularSpeed.z), glm::vec3(0,0,1));
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




float computeVelocityFactor(RigidBody* rbA, RigidBody* rbB, glm::vec3 normal) {
    return glm::dot(-(1 + rbA->bounce) * (rbA->velocity - rbB->velocity), normal) / (rbA->inverseOfMass + rbB->inverseOfMass);
}

float computeAngularFactor(RigidBody* rbA, RigidBody* rbB, glm::vec3 normal, glm::vec3 rA, glm::vec3 rB) {
    // couple = I * a = m * r^2 * a
    // I = m * r^2
    float IAinv = 1.0f / (rbA->mass * glm::dot(rA, rA));
    float IBinv = 1.0f / (rbB->mass * glm::dot(rB, rB));
    return glm::dot(-(1 + rbA->bounce) * (rbA->velocity - rbB->velocity), normal)
        / (rbA->inverseOfMass + rbB->inverseOfMass + glm::dot(glm::cross(IAinv * glm::cross(rA, normal), rA) + glm::cross(IBinv * glm::cross(rB, normal), rB), normal));
}





/*=========================================================================================================================*/
Collider* RigidBodySystem::getColliderFromEntityID(unsigned short entityID) {
    if (!EntityManager::instance->hasComponent(SystemIDs::ColliderID, entityID)) {
        // If entity doesn't have a ColliderID, return NULL
        return 0;
    }

    if (colliderForRotational.size() < entityID) {
        // If colliderForRotational is too small, extend it
        colliderForRotational.resize(entityID + 1, 0);
    }
    if(colliderSystem) return colliderSystem->getColliderEntityID(entityID);
    return 0;
}

void RigidBodySystem::ApplyForces(RigidBody* rb) {
    rb->combinedForces = gravity * rb->mass;
}

//Resolves constraints : PARTICLES
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


/*================ LINEAR MOVEMENT EQUATIONS ================*/



//Resolves constraints : VOLUMES (Colliders objects)
void RigidBodySystem::applyImpulse_linear(RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res, int nbC) {
    //std::cout << "APPLY IMPULSE LINEAR" << std::endl;
    //std::cout << "rb_1 : " << rb_1 << std::endl;
    //std::cout << "rb_2 : " << rb_2 << std::endl;
    //std::cout << "pres " << p_res << std::endl;
    //p_res->print();
    
    //std::cout << "rb_1 static : " << rb_1->static_RB << std::endl;
    //std::cout << "rb_2 static : " << rb_2->static_RB << std::endl;


    //We will apply an impulse reaction to perform ejections of both elem
    if (!rb_1->static_RB || !rb_2->static_RB) {
        //std::cout << "PERFORM AN IMPULSE" << std::endl;

        float invMassSum;
        if (rb_1->static_RB) {
            invMassSum = rb_2->inverseOfMass;
        }
        else if (rb_2->static_RB) {
            invMassSum = rb_1->inverseOfMass;
        }
        else {
            invMassSum = rb_1->inverseOfMass + rb_2->inverseOfMass;
        }

        if (invMassSum == 0.0f) return;


        glm::vec3 relative_vel = rb_1->velocity - rb_2->velocity;



        //std::cout << "RELATIVE VEL : ";
        //print(relative_vel);
        //std::cout << "VEL 1 : ";
        //print(rb_1->velocity);
        
        //std::cout << "VEL 2 : ";
        //print(rb_2->velocity);

        glm::vec3 relative_n = glm::normalize(p_res->normal);

        //std::cout << "RELATIVE NORMALE : ";
        //print(relative_n);


        //Magnitude of the relative velocity in the direction of the collision normal
        float d_vn = glm::dot(relative_vel, relative_n);

        //Moving away from each other
        if (d_vn > 0.0f) return;


        //Magnitude of our impulse
        float j_1 = 0.0f;
        float j_2 = 0.0f;
        if (!rb_1->static_RB) {
            float nume_1 = (-(1.0f + rb_1->bounce) * d_vn);
            j_1 = nume_1 / invMassSum;
        }
        if (!rb_2->static_RB) {
            float nume_2 = (-(1.0f + rb_2->bounce) * d_vn);
            j_2 = nume_2 / invMassSum;
        }


        if (nbC > 0 && j_1 != 0.0f) {
            j_1 /= (float)nbC;
        }
        if (nbC > 0 && j_2 != 0.0f) {
            j_2 /= (float)nbC;
        }

        if (!rb_1->static_RB) rb_1->velocity = rb_1->velocity + (j_1 * rb_1->inverseOfMass * relative_n);
        if (!rb_2->static_RB) rb_2->velocity = rb_2->velocity - (j_2 * rb_2->inverseOfMass * relative_n);
    }
    //std::cout << "FINAL VELOCITY : ";
    //print(rb_1->velocity);
    //print(rb_2->velocity);
    
    //std::cout << "====================================================" << std::endl;
}      
void RigidBodySystem::applyImpulse_linearFriction(RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res, int nbC) {
    if (DEBUG_RIGIDBODY) {
        std::cout << "APPLY IMPULSE LINEAR WITH FRICTIONS" << std::endl;
        std::cout << "rb_1 : " << rb_1 << std::endl;
        std::cout << "rb_2 : " << rb_2 << std::endl;
        std::cout << "pres " << p_res << std::endl;
        p_res->print();

        std::cout << "rb_1 static : " << rb_1->static_RB << std::endl;
        std::cout << "rb_2 static : " << rb_2->static_RB << std::endl;
    }


    //We will apply an impulse reaction to perform ejections of both elem
    if (!rb_1->static_RB || !rb_2->static_RB) {
        if (DEBUG_RIGIDBODY) {
            std::cout << "PERFORM AN IMPULSE" << std::endl;
        }

        float invMassSum;
        if (rb_1->static_RB) {
            invMassSum = rb_2->inverseOfMass;
        }
        else if (rb_2->static_RB) {
            invMassSum = rb_1->inverseOfMass;
        }
        else {
            invMassSum = rb_1->inverseOfMass + rb_2->inverseOfMass;
        }

        if (invMassSum == 0.0f) return;


        glm::vec3 relative_vel = rb_1->velocity - rb_2->velocity;



        if (DEBUG_RIGIDBODY) {
            std::cout << "RELATIVE VEL : ";
            print(relative_vel);
            std::cout << "VEL 1 : ";
            print(rb_1->velocity);

            std::cout << "VEL 2 : ";
            print(rb_2->velocity);
        }

        glm::vec3 relative_n = glm::normalize(p_res->normal);

        if (DEBUG_RIGIDBODY) {
            std::cout << "RELATIVE NORMALE : ";
            print(relative_n);
        }


        //Magnitude of the relative velocity in the direction of the collision normal
        float d_vn = glm::dot(relative_vel, relative_n);

        //Moving away from each other
        if (d_vn > 0.0f) return;


        //Magnitude of our impulse
        float j_1 = 0.0f;
        float j_2 = 0.0f;
        if (!rb_1->static_RB) {
            float nume_1 = (-(1.0f + rb_1->bounce) * d_vn);
            j_1 = nume_1 / invMassSum;
        }
        if (!rb_2->static_RB) {
            float nume_2 = (-(1.0f + rb_2->bounce) * d_vn);
            j_2 = nume_2 / invMassSum;
        }


        if (nbC > 0 && j_1 != 0.0f) {
            j_1 /= (float)nbC;
        }
        if (nbC > 0 && j_2 != 0.0f) {
            j_2 /= (float)nbC;
        }

        if (!rb_1->static_RB) rb_1->velocity = rb_1->velocity + (j_1 * rb_1->inverseOfMass * relative_n);
        if (!rb_2->static_RB) rb_2->velocity = rb_2->velocity - (j_2 * rb_2->inverseOfMass * relative_n);

        //Applying friction now
        //t : tangential vector to the collision normal
        glm::vec3 t = relative_vel - (relative_n * d_vn);

        if (DEBUG_RIGIDBODY) {
            std::cout << "dot vn : " << d_vn << std::endl;
            std::cout << "tangential vector : ";
            print(t);
        }

        if (compareWithEpsilon(glm::dot(t, t), 0.0f)) {
            return;
        }
        t = glm::normalize(t);

        //Find magnitude of friction to apply
        float numerator = -glm::dot(relative_vel, t);
        float jt = numerator / invMassSum;

        if (nbC > 0 && jt != 0.0f) {
            jt /= (float)nbC;
        }

        if (compareWithEpsilon(jt, 0.0f)) {
            return;
        }

        
        //Coulomb's law for friction :
        //The magnitude of friction can never be greater than or smaller than j scaled by the Coefficient of Friction, which is the square root of the product of the Coefficient of friction for both the objects
        float friction = sqrtf(rb_1->cineticFriction * rb_2->cineticFriction);

        if (!rb_1->static_RB) {
            float jt_1 = jt;
            
            if (jt_1 > j_1 * friction) {
                jt_1 = j_1 * friction;
            }
            else if (jt_1 < -j_1 * friction) {
                jt_1 = -j_1 * friction;
            }

            //Finally add tangent impulse friction to the velocity of components
            glm::vec3 tangentImpuse_1 = t * jt_1;
            rb_1->velocity = rb_1->velocity + tangentImpuse_1 * rb_1->inverseOfMass;
        }
        if (!rb_2->static_RB) {
            float jt_2 = jt;
            if (jt_2 > j_2 * friction) {
                jt_2 = j_2 * friction;
            }
            else if (jt_2 < -j_2 * friction) {
                jt_2 = -j_2 * friction;
            }

            //Finally add tangent impulse friction to the velocity of components
            glm::vec3 tangentImpuse_2 = t * jt_2;
            rb_2->velocity = rb_2->velocity - tangentImpuse_2 * rb_2->inverseOfMass;
        }
    }

    if (DEBUG_RIGIDBODY) {
        std::cout << "FINAL VELOCITY : ";
        print(rb_1->velocity);
        print(rb_2->velocity);

        std::cout << "====================================================" << std::endl;
    }
}
void RigidBodySystem::applyImpulse_linearDynamicFriction(RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res, int nbC) {
    if (DEBUG_RIGIDBODY) {
        std::cout << "APPLY IMPULSE LINEAR WITH DYNAMICS FRICTIONS" << std::endl;
        std::cout << "rb_1 : " << rb_1 << std::endl;
        std::cout << "rb_2 : " << rb_2 << std::endl;
        std::cout << "pres " << p_res << std::endl;
        p_res->print();

        std::cout << "rb_1 static : " << rb_1->static_RB << std::endl;
        std::cout << "rb_2 static : " << rb_2->static_RB << std::endl;
    }

    //We will apply an impulse reaction to perform ejections of both elem
    if (!rb_1->static_RB || !rb_2->static_RB) {
        if (DEBUG_RIGIDBODY) {
            std::cout << "PERFORM AN IMPULSE" << std::endl;
        }

        float invMassSum;
        if (rb_1->static_RB) {
            invMassSum = rb_2->inverseOfMass;
        }
        else if (rb_2->static_RB) {
            invMassSum = rb_1->inverseOfMass;
        }
        else {
            invMassSum = rb_1->inverseOfMass + rb_2->inverseOfMass;
        }

        if (invMassSum == 0.0f) return;


        glm::vec3 relative_vel = rb_1->velocity - rb_2->velocity;



        if (DEBUG_RIGIDBODY) {
            std::cout << "RELATIVE VEL : ";
            print(relative_vel);
            std::cout << "VEL 1 : ";
            print(rb_1->velocity);

            std::cout << "VEL 2 : ";
            print(rb_2->velocity);
        }

        glm::vec3 relative_n = glm::normalize(p_res->normal);

        if (DEBUG_RIGIDBODY) {
            std::cout << "RELATIVE NORMALE : ";
            print(relative_n);
        }


        //Magnitude of the relative velocity in the direction of the collision normal
        float d_vn = glm::dot(relative_vel, relative_n);

        //Moving away from each other
        if (d_vn > 0.0f) return;


        //Magnitude of our impulse
        float j_1 = 0.0f;
        float j_2 = 0.0f;
        if (!rb_1->static_RB) {
            float nume_1 = (-(1.0f + rb_1->bounce) * d_vn);
            j_1 = nume_1 / invMassSum;
        }
        if (!rb_2->static_RB) {
            float nume_2 = (-(1.0f + rb_2->bounce) * d_vn);
            j_2 = nume_2 / invMassSum;
        }


        if (nbC > 0 && j_1 != 0.0f) {
            j_1 /= (float)nbC;
        }
        if (nbC > 0 && j_2 != 0.0f) {
            j_2 /= (float)nbC;
        }

        if (!rb_1->static_RB) rb_1->velocity = rb_1->velocity + (j_1 * rb_1->inverseOfMass * relative_n);
        if (!rb_2->static_RB) rb_2->velocity = rb_2->velocity - (j_2 * rb_2->inverseOfMass * relative_n);
        
        //Applying friction now
        //t : tangential vector to the collision normal
        glm::vec3 t = relative_vel - (relative_n * d_vn);

        if (compareWithEpsilon(glm::dot(t, t), 0.0f)) {
            return;
        }
        t = glm::normalize(t);

        //Find magnitude of friction to apply
        float numerator = -glm::dot(relative_vel, t);
        float jt = numerator / invMassSum;

        if (nbC > 0 && jt != 0.0f) {
            jt /= (float)nbC;
        }

        if (compareWithEpsilon(jt, 0.0f)) {
            return;
        }

        glm::vec3 tangentImpuse_1;
        glm::vec3 tangentImpuse_2;
        float sf = sqrtf(rb_1->staticFriction * rb_2->staticFriction);
        float df = sqrtf(rb_1->cineticFriction * rb_1->cineticFriction);
        if (!rb_1->static_RB) {
            if (fabsf(jt) < j_1 * sf) {
                tangentImpuse_1 = t * jt;
            }
            else {
                tangentImpuse_1 = t * -j_1 * df;
            }
            rb_1->velocity = rb_1->velocity - tangentImpuse_1 * rb_1->inverseOfMass;
        }
        if (!rb_2->static_RB) {

            if (fabsf(jt) < j_2 * sf) {
                tangentImpuse_2 = t * jt;
            }
            else {
                tangentImpuse_2 = t * -j_2 * df;
            }
            rb_2->velocity = rb_2->velocity + tangentImpuse_2 * rb_2->inverseOfMass;
        }
    }

    if (DEBUG_RIGIDBODY) {
        std::cout << "FINAL VELOCITY : ";
        print(rb_1->velocity);
        print(rb_2->velocity);

        std::cout << "====================================================" << std::endl;
    }
}

//Correct new RB position
void RigidBodySystem::positionCorrection(glm::vec3& newPos1, glm::vec3& newPos2, RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res) {
    //std::cout << "      ====== = POSITION CORRECTION" << std::endl;
    //print(p_res->normal);
    

    if (!rb_1->static_RB || !rb_2->static_RB) {

        float totalMass;
        if (rb_1->static_RB) {
            totalMass = rb_2->inverseOfMass;
        }
        else if (rb_2->static_RB) {
            totalMass = rb_1->inverseOfMass;
        }
        else {
            totalMass = rb_1->inverseOfMass + rb_2->inverseOfMass;
        }

        if (totalMass == 0.0f) {
            return;
        }

        float depth = fmaxf(p_res->penetrationDistance - penetrationSlack, 0.0f);
        float scalar = depth / totalMass;
        glm::vec3 correction = p_res->normal * scalar * linearProjectionPercent;

        if (!rb_1->static_RB) newPos1 = newPos1 + correction * rb_1->inverseOfMass;
        if (!rb_2->static_RB) newPos2 = newPos2 - correction * rb_2->inverseOfMass;
    }
    //std::cout << "=========================================" << std::endl;

}


/*================ ROTATIONAL MOVEMENT EQUATIONS ================*/
//For the rotation, we got two different types of acceleration (and velocity) to take in care :
//  Tangential Acceleration
//  This acceleration will change the magnitude of our velocity
//  Centripetal acceleration, happens around the <CENTER OF MASS>
//  This acceleration will change the direction of the object  velocity, but not the magnitude

//  Torque : The further a point where force is applied is from the center of mass of an object, the less force it takes to rotate the object
//  To find the total amount of torque applied in an object, we need to sum up torque
//  sum(torque) = sum(r²m) * a
//  where sum(r²m) is the <<moment of inertia>>, "I". This vakue is a 3x3 mat, different for every shapes
TensorMatrix::TensorMatrix() {
    computed = false;
    mat = glm::mat4(1.0f);
}
TensorMatrix::~TensorMatrix() {}

void RigidBodySystem::clearMatrixTensorStructure(unsigned short id) {
    tensorMatrix[id].computed = false;
}
glm::mat4 RigidBodySystem::getMatrixInverseTensor(unsigned short entityID, Collider c, RigidBody* rb) {

    if (tensorMatrix.size() <= entityID) {
        // If tensorMatrix is too small, extend it
        //std::cout << " -> resize tensor matrix" << std::endl;
        tensorMatrix.resize(entityID + 1, TensorMatrix());
    }

    //std::cout << "compute tensor matrix from : " << entityID << std::endl;
    //std::cout << "size : " << tensorMatrix.size() << std::endl;

    if (!tensorMatrix[entityID].computed) {
        //std::cout << "=========================" << std::endl;
        //std::cout << "GET MATRIX INVERSE TENSOR : " << entityID << std::endl;

        tensorMatrix[entityID].computed = true;
        tensorMatrix[entityID].mat = inverseTensorComputation(c, rb);
        return tensorMatrix[entityID].mat;
    }
    else {
        return tensorMatrix[entityID].mat;
    }
}


void RigidBody::rotational_addImpulseAtPosition(const glm::vec3& point, const glm::vec3& impulse, const glm::vec3& currentPosition, const glm::mat3& inverseTensor) {
    glm::vec3 centerOfMass = currentPosition;
    glm::vec3 torque = glm::cross(point - centerOfMass, impulse);
    glm::vec3 angAccel = torque *inverseTensor;
    angularSpeed += angAccel;
}

// Inertia tensor is the equivalent of mass for linear speed computation
glm::mat4 RigidBodySystem::inverseTensorComputation(Collider c, RigidBody* rb) {
    if (rb->mass == 0) {
        return glm::mat4(0);
    }


    float ix = 0.0f;
    float iy = 0.0f;
    float iz = 0.0f;
    //float iw = 0.0f;

    if (c.type == colliderType::Sphere) {
        //std::cout << "COMPUTE TENSOR MAT SPHERE" << std::endl;
        //std::cout << "->RADIUS : "  << c.radius << std::endl;
        //printRB(rb);

        float r2 = c.radius * c.radius;
        float fraction = (2.0f / 5.0f);

        ix = r2 * rb->mass * fraction;
        iy = r2 * rb->mass * fraction;
        iz = r2 * rb->mass * fraction;
        //iw = 1.0f;
    }
    else if (c.type == colliderType::AABB || c.type == colliderType::OBB) {
        //std::cout << "COMPUTE TENSOR MAT BOX" << std::endl;
        //print(c.dimensions);
        //printRB(rb);

        glm::vec3 size = c.dimensions * 2.0f;
        float fraction = (1.0f / 12.0f);

        float x2 = size.x * size.x;
        float y2 = size.y * size.y;
        float z2 = size.z * size.z;

        ix = (y2 + z2) * rb->mass * fraction;
        iy = (x2 + z2) * rb->mass * fraction;
        iz = (x2 + y2) * rb->mass * fraction;
        //iw = 1.0f;
    }

    return glm::mat4(
        1.0f / ix, 0, 0, 0,
        0, 1.0f / iy, 0, 0,
        0, 0, 1.0f / iz, 0,
        0, 0, 0, 1.0f);
}



// TOTAL VELOCITY : (linear plus angular) by adding the rotational velocity to the 
//  Linear Velocity of the rigidbody at the center of mass. We also need to find the torque from
//  the point of impactand collision normal divided by the inertia tensor.Knowing this, we can
//  find the final equation for j
void RigidBodySystem::applyImpulse_linearAngular(RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res, int nbC, const glm::vec3& posrb1, const glm::vec3& posrb2, const glm::mat4& inverseTensorrb1, const glm::mat4& inverseTensorrb2) {
    if (DEBUG_RIGIDBODY) {
        std::cout << "APPLY IMPULSE LINEAR ANGULAR" << std::endl;
        std::cout << "rb_1 : " << rb_1 << std::endl;
        std::cout << "rb_2 : " << rb_2 << std::endl;
        std::cout << "pres " << p_res << std::endl;
        p_res->print();
        
        std::cout << "rb_1 static : " << rb_1->static_RB << std::endl;
        std::cout << "rb_2 static : " << rb_2->static_RB << std::endl;


        print(posrb1);
        print(posrb2);
        print(inverseTensorrb1);
        print(inverseTensorrb2);
    }

    //We will apply an impulse reaction to perform ejections of both elem
    if (!rb_1->static_RB || !rb_2->static_RB) {
        //std::cout << "PERFORM AN IMPULSE" << std::endl;

        float invMassSum;
        if (rb_1->static_RB) {
            invMassSum = rb_2->inverseOfMass;
        }
        else if (rb_2->static_RB) {
            invMassSum = rb_1->inverseOfMass;
        }
        else {
            invMassSum = rb_1->inverseOfMass + rb_2->inverseOfMass;
        }

        if (invMassSum == 0.0f) return;


        glm::vec3 r1 = p_res->point - posrb1;
        glm::vec3 r2 = p_res->point - posrb2;

        //std::cout << "RELATIVE R1 : ";
        //print(r1);
        //std::cout << "RELATIVE R2 : ";
        //print(r2);


        glm::mat4 i1 = inverseTensorrb1;
        glm::mat4 i2 = inverseTensorrb2;

        //The cross product of angular velocity and the relative contact point will give us the magnitude of rotational velocity
        //glm::vec3 relative_vel = (rb_1->velocity + glm::cross(rb_1->angularSpeed, r1)) - (rb_2->angularSpeed + glm::cross(rb_2->angularSpeed, r2));
        //glm::vec3 relative_vel = (rb_1->oldvelocity + glm::cross(rb_1->angularSpeed, r1)) - (rb_2->oldvelocity + glm::cross(rb_2->angularSpeed, r2));
        //glm::vec3 relative_vel = (rb_2->velocity + glm::cross(rb_2->angularSpeed, r2)) - (rb_1->angularSpeed + glm::cross(rb_1->angularSpeed, r1));
        
        glm::vec3 relative_vel = rb_1->oldvelocity - rb_2->oldvelocity;


        //std::cout << "RELATIVE VEL : ";
        //print(relative_vel);
        //std::cout << "VEL 1 : ";
        //print(rb_1->velocity);
        //
        //std::cout << "VEL 2 : ";
        //print(rb_2->velocity);

        glm::vec3 relative_n = glm::normalize(p_res->normal);

        //std::cout << "RELATIVE NORMALE : ";
        //print(relative_n);


        //Magnitude of the relative velocity in the direction of the collision normal
        float d_vn = glm::dot(relative_vel, relative_n);

        //Moving away from each other
        if (d_vn > 0.0f) return;

        float d1 = invMassSum;
        glm::vec4 resd_1 = glm::vec4(glm::cross(r1, relative_n), 0.0f) * i1;
        glm::vec4 resd_2 = glm::vec4(glm::cross(r2, relative_n), 0.0f) * i2;

        glm::vec3 d2 = glm::cross(glm::vec3(resd_1.x, resd_1.y, resd_1.z), r1);
        glm::vec3 d3 = glm::cross(glm::vec3(resd_2.x, resd_2.y, resd_2.z), r2);
        float denominator = d1 + glm::dot(relative_n, d2 + d3);


        //Magnitude of our impulse
        float j_1 = 0.0f;
        float j_2 = 0.0f;

        if (!rb_1->static_RB && denominator != 0) {
            float nume_1 = (-(1.0f + rb_1->bounce) * d_vn);
            j_1 = nume_1 / denominator;
        }
        if (!rb_2->static_RB && denominator != 0) {
            float nume_2 = (-(1.0f + rb_2->bounce) * d_vn);
            j_2 = nume_2 / denominator;
        }


        if (nbC > 0 && j_1 != 0.0f) {
            j_1 /= (float)nbC;
        }
        if (nbC > 0 && j_2 != 0.0f) {
            j_2 /= (float)nbC;
        }


        if (!rb_1->static_RB) {
            glm::vec3 impulse_1 = j_1 * relative_n;
            //std::cout << "RB1 VELOCITY : " << j_1 << std::endl;
            //print(impulse_1);
            //print(rb_1->velocity);

            rb_1->velocity = rb_1->velocity + (impulse_1 * rb_1->inverseOfMass );

            //print(rb_1->velocity);

            //Update ANGULAR VELOCITY
            glm::vec4 res = glm::vec4(glm::cross(r1, impulse_1), 0.0f) * i1;
            rb_1->angularSpeed = rb_1->angularSpeed - glm::vec3(res.x, res.y, res.z);
        }
        if (!rb_2->static_RB) {
            glm::vec3 impulse_2 = j_2 * relative_n;
            //std::cout << "RB2 VELOCITY : " << j_2 << std::endl;
            //print(impulse_2);
            

            rb_2->velocity = rb_2->velocity - (impulse_2 * rb_2->inverseOfMass);

            //Update ANGULAR VELOCITY
            glm::vec4 res = glm::vec4(glm::cross(r2, impulse_2), 0.0f) * i2;
            rb_2->angularSpeed = rb_2->angularSpeed + glm::vec3(res.x, res.y, res.z);
        }
    }

    if (DEBUG_RIGIDBODY) {
        std::cout << "FINAL VELOCITY : ";
        print(rb_1->velocity);
        print(rb_2->velocity);
        std::cout << "FINAL ANGULAR VELOCITY : ";
        print(rb_1->angularSpeed);
        print(rb_2->angularSpeed);
        std::cout << "====================================================" << std::endl;
    }
}
void RigidBodySystem::applyImpulse_linearAngularFriction(RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res, int nbC, const glm::vec3& posrb1, const glm::vec3& posrb2, const glm::mat4& inverseTensorrb1, const glm::mat4& inverseTensorrb2) {
    if (DEBUG_RIGIDBODY) {
        std::cout << "APPLY IMPULSE LINEAR ANGULAR FRICTION" << std::endl;
        std::cout << "rb_1 : " << rb_1 << std::endl;
        std::cout << "rb_2 : " << rb_2 << std::endl;
        std::cout << "pres " << p_res << std::endl;
        p_res->print();

        std::cout << "rb_1 static : " << rb_1->static_RB << std::endl;
        std::cout << "rb_2 static : " << rb_2->static_RB << std::endl;


        print(posrb1);
        print(posrb2);
        print(inverseTensorrb1);
        print(inverseTensorrb2);
    }

    //We will apply an impulse reaction to perform ejections of both elem
    if (!rb_1->static_RB || !rb_2->static_RB) {
        if (DEBUG_RIGIDBODY) {
            std::cout << "PERFORM AN IMPULSE" << std::endl;
        }

        float invMassSum;
        if (rb_1->static_RB) {
            invMassSum = rb_2->inverseOfMass;
        }
        else if (rb_2->static_RB) {
            invMassSum = rb_1->inverseOfMass;
        }
        else {
            invMassSum = rb_1->inverseOfMass + rb_2->inverseOfMass;
        }

        if (invMassSum == 0.0f) return;


        glm::vec3 r1 = p_res->point - posrb1;
        glm::vec3 r2 = p_res->point - posrb2;

        glm::mat4 i1 = inverseTensorrb1;
        glm::mat4 i2 = inverseTensorrb2;

        //The cross product of angular velocity and the relative contact point will give us the magnitude of rotational velocity
        glm::vec3 relative_vel = (rb_2->velocity + glm::cross(rb_2->angularSpeed, r2)) - (rb_1->angularSpeed + glm::cross(rb_1->angularSpeed, r1));


        if (DEBUG_RIGIDBODY) {
            std::cout << "RELATIVE VEL : ";
            print(relative_vel);
            std::cout << "VEL 1 : ";
            print(rb_1->velocity);

            std::cout << "VEL 2 : ";
            print(rb_2->velocity);
        }

        glm::vec3 relative_n = glm::normalize(p_res->normal);

        if (DEBUG_RIGIDBODY) {
            std::cout << "RELATIVE NORMALE : ";
            print(relative_n);
        }


        //Magnitude of the relative velocity in the direction of the collision normal
        float d_vn = glm::dot(relative_vel, relative_n);

        //Moving away from each other
        if (d_vn > 0.0f) return;

        float d1 = invMassSum;
        glm::vec4 resd_1 = glm::vec4(glm::cross(r1, relative_n), 0.0f) * i1;
        glm::vec4 resd_2 = glm::vec4(glm::cross(r2, relative_n), 0.0f) * i2;

        glm::vec3 d2 = glm::cross(glm::vec3(resd_1.x, resd_1.y, resd_1.z), r1);
        glm::vec3 d3 = glm::cross(glm::vec3(resd_2.x, resd_2.y, resd_2.z), r2);
        float denominator = d1 + glm::dot(relative_n, d2 + d3);


        //Magnitude of our impulse
        float j_1 = 0.0f;
        float j_2 = 0.0f;
        if (!rb_1->static_RB) {
            float nume_1 = (-(1.0f + rb_1->bounce) * d_vn);
            j_1 = nume_1 / denominator;
        }
        if (!rb_2->static_RB) {
            float nume_2 = (-(1.0f + rb_2->bounce) * d_vn);
            j_2 = nume_2 / denominator;
        }


        if (nbC > 0 && j_1 != 0.0f) {
            j_1 /= (float)nbC;
        }
        if (nbC > 0 && j_2 != 0.0f) {
            j_2 /= (float)nbC;
        }

        if (!rb_1->static_RB) {
            glm::vec3 impulse_1 = j_1 * relative_n;
            rb_1->velocity = rb_1->velocity + (impulse_1 * rb_1->inverseOfMass);

            //Update ANGULAR VELOCITY
            glm::vec4 res = glm::vec4(glm::cross(r1, impulse_1), 0.0f) * i1;
            rb_1->angularSpeed = rb_1->angularSpeed + glm::vec3(res.x, res.y, res.z);
        }
        if (!rb_2->static_RB) {
            glm::vec3 impulse_2 = j_2 * relative_n;
            rb_2->velocity = rb_2->velocity - (impulse_2 * relative_n);

            //Update ANGULAR VELOCITY
            glm::vec4 res = glm::vec4(glm::cross(r2, impulse_2), 0.0f) * i2;
            rb_2->angularSpeed = rb_2->angularSpeed - glm::vec3(res.x, res.y, res.z);
        }


        //Coulomb's law for friction :
        //The magnitude of friction can never be greater than or smaller than j scaled by the Coefficient of Friction, which is the square root of the product of the Coefficient of friction for both the objects
        //t : tangential vector to the collision normal
        glm::vec3 t = relative_vel - (relative_n * d_vn);

        if (compareWithEpsilon(glm::dot(t, t), 0.0f)) {
            return;
        }
        t = glm::normalize(t);

        //Find magnitude of friction to apply
        float numerator = -glm::dot(relative_vel, t);
        d1 = invMassSum;

        resd_1 = glm::vec4(glm::cross(r1, t), 0.0f) * i1;
        resd_2 = glm::vec4(glm::cross(r2, t), 0.0f) * i2;

        d2 = glm::cross(glm::vec3(resd_1.x, resd_1.y, resd_1.z), r1);
        d3 = glm::cross(glm::vec3(resd_2.x, resd_2.y, resd_2.z), r2);
        denominator = d1 + glm::dot(t, d2 + d3);

        if (denominator == 0.0f) {
            return;
        }

        float jt = numerator / invMassSum;
        if (nbC > 0 && jt != 0.0f) {
            jt /= (float)nbC;
        }

        if (compareWithEpsilon(jt, 0.0f)) {
            return;
        }
        

        float friction = sqrtf(rb_1->cineticFriction * rb_2->cineticFriction);

        if (!rb_1->static_RB) {
            float jt_1 = jt;

            if (jt_1 > j_1 * friction) {
                jt_1 = j_1 * friction;
            }
            else if (jt_1 < -j_1 * friction) {
                jt_1 = -j_1 * friction;
            }


            //Finally add tangent impulse friction to the velocity of components
            glm::vec3 tangentImpuse_1 = t * jt_1;
            rb_1->velocity = rb_1->velocity + tangentImpuse_1 * rb_1->inverseOfMass;

            glm::vec4 res = glm::vec4(glm::cross(r1, tangentImpuse_1), 0.0f) * i1;
            rb_1->angularSpeed = rb_1->angularSpeed + glm::vec3(res.x, res.y, res.z);
        }
        if (!rb_2->static_RB) {
            float jt_2 = jt;
            if (jt_2 > j_2 * friction) {
                jt_2 = j_2 * friction;
            }
            else if (jt_2 < -j_2 * friction) {
                jt_2 = -j_2 * friction;
            }

            //Finally add tangent impulse friction to the velocity of components
            glm::vec3 tangentImpuse_2 = t * jt_2;
            rb_2->velocity = rb_2->velocity - tangentImpuse_2 * rb_2->inverseOfMass;

            glm::vec4 res = glm::vec4(glm::cross(r2, tangentImpuse_2), 0.0f) * i2;
            rb_2->angularSpeed = rb_2->angularSpeed - glm::vec3(res.x, res.y, res.z);
        }
    }
}
void RigidBodySystem::applyImpulse_linearAngularDynamicFriction(RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res, int nbC, const glm::vec3& posrb1, const glm::vec3& posrb2, const glm::mat4& inverseTensorrb1, const glm::mat4& inverseTensorrb2) {
    if (DEBUG_RIGIDBODY) {
        std::cout << "APPLY IMPULSE LINEAR ANGULAR DYNAMIC FRICTION" << std::endl;
        std::cout << "rb_1 : " << rb_1 << std::endl;
        std::cout << "rb_2 : " << rb_2 << std::endl;
        std::cout << "pres " << p_res << std::endl;
        p_res->print();

        std::cout << "rb_1 static : " << rb_1->static_RB << std::endl;
        std::cout << "rb_2 static : " << rb_2->static_RB << std::endl;


        print(posrb1);
        print(posrb2);
        print(inverseTensorrb1);
        print(inverseTensorrb2);
    }

    //We will apply an impulse reaction to perform ejections of both elem
    if (!rb_1->static_RB || !rb_2->static_RB) {
        if (DEBUG_RIGIDBODY) {
            std::cout << "PERFORM AN IMPULSE" << std::endl;
        }

        float invMassSum;
        if (rb_1->static_RB) {
            invMassSum = rb_2->inverseOfMass;
        }
        else if (rb_2->static_RB) {
            invMassSum = rb_1->inverseOfMass;
        }
        else {
            invMassSum = rb_1->inverseOfMass + rb_2->inverseOfMass;
        }

        if (invMassSum == 0.0f) return;


        glm::vec3 r1 = p_res->point - posrb1;
        glm::vec3 r2 = p_res->point - posrb2;

        glm::mat4 i1 = inverseTensorrb1;
        glm::mat4 i2 = inverseTensorrb2;

        //The cross product of angular velocity and the relative contact point will give us the magnitude of rotational velocity
        glm::vec3 relative_vel = (rb_2->velocity + glm::cross(rb_2->angularSpeed, r2)) - (rb_1->angularSpeed + glm::cross(rb_1->angularSpeed, r1));


        if (DEBUG_RIGIDBODY) {
            std::cout << "RELATIVE VEL : ";
            print(relative_vel);
            std::cout << "VEL 1 : ";
            print(rb_1->velocity);

            std::cout << "VEL 2 : ";
            print(rb_2->velocity);
        }

        glm::vec3 relative_n = glm::normalize(p_res->normal);

        if (DEBUG_RIGIDBODY) {
            std::cout << "RELATIVE NORMALE : ";
            print(relative_n);
        }


        //Magnitude of the relative velocity in the direction of the collision normal
        float d_vn = glm::dot(relative_vel, relative_n);

        //Moving away from each other
        if (d_vn > 0.0f) return;

        float d1 = invMassSum;
        glm::vec4 resd_1 = glm::vec4(glm::cross(r1, relative_n), 0.0f) * i1;
        glm::vec4 resd_2 = glm::vec4(glm::cross(r2, relative_n), 0.0f) * i2;

        glm::vec3 d2 = glm::cross(glm::vec3(resd_1.x, resd_1.y, resd_1.z), r1);
        glm::vec3 d3 = glm::cross(glm::vec3(resd_2.x, resd_2.y, resd_2.z), r2);
        float denominator = d1 + glm::dot(relative_n, d2 + d3);


        //Magnitude of our impulse
        float j_1 = 0.0f;
        float j_2 = 0.0f;
        if (!rb_1->static_RB) {
            float nume_1 = (-(1.0f + rb_1->bounce) * d_vn);
            j_1 = nume_1 / denominator;
        }
        if (!rb_2->static_RB) {
            float nume_2 = (-(1.0f + rb_2->bounce) * d_vn);
            j_2 = nume_2 / denominator;
        }


        if (nbC > 0 && j_1 != 0.0f) {
            j_1 /= (float)nbC;
        }
        if (nbC > 0 && j_2 != 0.0f) {
            j_2 /= (float)nbC;
        }

        if (!rb_1->static_RB) {
            glm::vec3 impulse_1 = j_1 * relative_n;
            rb_1->velocity = rb_1->velocity + (impulse_1 * rb_1->inverseOfMass);

            //Update ANGULAR VELOCITY
            glm::vec4 res = glm::vec4(glm::cross(r1, impulse_1), 0.0f) * i1;
            rb_1->angularSpeed = rb_1->angularSpeed + glm::vec3(res.x, res.y, res.z);
        }
        if (!rb_2->static_RB) {
            glm::vec3 impulse_2 = j_2 * relative_n;
            rb_2->velocity = rb_2->velocity - (impulse_2 * relative_n);

            //Update ANGULAR VELOCITY
            glm::vec4 res = glm::vec4(glm::cross(r2, impulse_2), 0.0f) * i2;
            rb_2->angularSpeed = rb_2->angularSpeed - glm::vec3(res.x, res.y, res.z);
        }


        //Coulomb's law for friction :
        //The magnitude of friction can never be greater than or smaller than j scaled by the Coefficient of Friction, which is the square root of the product of the Coefficient of friction for both the objects
        //t : tangential vector to the collision normal
        glm::vec3 t = relative_vel - (relative_n * d_vn);

        if (compareWithEpsilon(glm::dot(t, t), 0.0f)) {
            return;
        }
        t = glm::normalize(t);

        //Find magnitude of friction to apply
        float numerator = -glm::dot(relative_vel, t);
        d1 = invMassSum;

        resd_1 = glm::vec4(glm::cross(r1, t), 0.0f) * i1;
        resd_2 = glm::vec4(glm::cross(r2, t), 0.0f) * i2;

        d2 = glm::cross(glm::vec3(resd_1.x, resd_1.y, resd_1.z), r1);
        d3 = glm::cross(glm::vec3(resd_2.x, resd_2.y, resd_2.z), r2);
        denominator = d1 + glm::dot(t, d2 + d3);

        if (denominator == 0.0f) {
            return;
        }

        float jt = numerator / invMassSum;
        if (nbC > 0 && jt != 0.0f) {
            jt /= (float)nbC;
        }

        if (compareWithEpsilon(jt, 0.0f)) {
            return;
        }


        float sf = sqrtf(rb_1->staticFriction * rb_2->staticFriction);
        float df = sqrtf(rb_1->cineticFriction * rb_1->cineticFriction);

        if (!rb_1->static_RB) {
            //Finally add tangent impulse friction to the velocity of components
            glm::vec3 tangentImpuse_1;
            if (fabsf(jt) < j_1 * sf) {
                tangentImpuse_1 = t * jt;
            }
            else {
                tangentImpuse_1 = t * -j_1 * df;
            }
            

            rb_1->velocity = rb_1->velocity + tangentImpuse_1 * rb_1->inverseOfMass;

            glm::vec4 res = glm::vec4(glm::cross(r1, tangentImpuse_1), 0.0f) * i1;
            rb_1->angularSpeed = rb_1->angularSpeed + glm::vec3(res.x, res.y, res.z);
        }
        if (!rb_2->static_RB) {
            glm::vec3 tangentImpuse_2;
            if (fabsf(jt) < j_2 * sf) {
                tangentImpuse_2 = t * jt;
            }
            else {
                tangentImpuse_2 = t * -j_2 * df;
            }

            //Finally add tangent impulse friction to the velocity of components
            rb_2->velocity = rb_2->velocity - tangentImpuse_2 * rb_2->inverseOfMass;

            glm::vec4 res = glm::vec4(glm::cross(r2, tangentImpuse_2), 0.0f) * i2;
            rb_2->angularSpeed = rb_2->angularSpeed - glm::vec3(res.x, res.y, res.z);
        }
    }
}


void RigidBodySystem::applyImpulse_linearAngular_bis(RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res, int nbC, const glm::vec3& posrb1, const glm::vec3& posrb2, const glm::mat4& inverseTensorrb1, const glm::mat4& inverseTensorrb2) {
    if (DEBUG_RIGIDBODY) {
        std::cout << "APLLY LINEAR ANGULAR BIS" << std::endl;
    }
    applyImpulse_linear(rb_1, rb_2, p_res, nbC);
    if(!rb_1->static_RB) rb_1->rotational_addImpulseAtPosition(p_res->point, p_res->normal * p_res->penetrationDistance, posrb1, getFromMat4(inverseTensorrb1));
    if(!rb_2->static_RB) rb_2->rotational_addImpulseAtPosition(p_res->point, -p_res->normal * p_res->penetrationDistance, posrb2, getFromMat4(inverseTensorrb2));
}



void RigidBodySystem::applySpringConnection(RigidBody* rb_1, RigidBody* rb_2, const glm::vec3 pos_1, const glm::vec3 pos_2, float deltaTime, float restingLength, float k_springStrenght, float b_springFriction) {
    glm::vec3 relPos = pos_2 - pos_1;
    glm::vec3 relVel = rb_2->velocity - rb_1->velocity;

    // Prevent underflow
    for (int i = 0; i < 3; ++i) {
        relPos[i] = (fabsf(relPos[i]) < 0.0000001f) ? 0.0f : relPos[i];
        relVel[i] = (fabsf(relVel[i]) < 0.0000001f) ? 0.0f : relVel[i];
    }

    float x = glm::length(relPos) - restingLength;
    float v = glm::length(relVel);

    float F = (-k_springStrenght * x) + (-b_springFriction * v);

    glm::vec3 impulse = glm::normalize(relPos) * F;
    rb_1->addImpulse(impulse * rb_1->inverseOfMass);
    rb_2->addImpulse(impulse * -1.0f * rb_2->inverseOfMass);
}
void RigidBodySystem::resolveConstraintParticles_JointDistance(RigidBody* rb_particles_1, RigidBody* rb_particles_2, glm::vec3& currentPos_1, glm::vec3& currentPos_2, float distancejoint) {
    glm::vec3 delta = currentPos_2 - currentPos_1;
    float distance = glm::length(delta);
    float correction = (distance - distancejoint) / distance;

    currentPos_1 += delta * 0.5f * correction;
    currentPos_2 -= delta * 0.5f * correction;
}










/*
void RigidBodySystem::linearAndAngular_resolveConstraintVolumeRB_Euler(RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res, int nbC, const glm::vec3& posrb1, const glm::vec3& posrb2, const glm::mat4& inverseTensorrb1, const glm::mat4& inverseTensorrb2) {
    //std::cout << "LINEAR AND ANGULAR RESOLVE CONSTRAINT VOLUME RB" << std::endl;
    //std::cout << "rb_1 : " << rb_1 << std::endl;
    //std::cout << "rb_2 : " << rb_2 << std::endl;
    
    //std::cout << "pres " << p_res << std::endl;
    //p_res->print();
    
    //std::cout << "rb_1 static : " << rb_1->static_RB << std::endl;
    //std::cout << "rb_2 static : " << rb_2->static_RB << std::endl;
    
    //print(posrb1);
    //print(posrb2);
    
    //print(inverseTensorrb1);
    //print(inverseTensorrb2);

    //We will apply an impulse reaction to perform ejections of both elem
    if (!rb_1->static_RB || !rb_2->static_RB) {
        //std::cout << "PERFORM CALCS" << std::endl;

        float invMassSum = rb_1->inverseOfMass + rb_2->inverseOfMass;
        if (invMassSum == 0.0f) return;

        glm::vec3 r1 = p_res->point - posrb1;
        glm::vec3 r2 = p_res->point - posrb2;

        glm::mat4 i1 = inverseTensorrb1;
        glm::mat4 i2 = inverseTensorrb2;


        
        // Relative collision normal
        glm::vec3 relative_n = glm::normalize(p_res->normal);

        //Magnitude of the relative velocity in the direction of the collision normal
        float d_vn = glm::dot(relative_vel, relative_n);

        // Moving away from each other? Do nothing!
        if (d_vn > 0.0f) {
            return;
        }

        float e = fminf(rb_1->bounce, rb_2->bounce);
        float numerator = (-(1.0f + e) * d_vn);
        float d1 = invMassSum;

        glm::vec4 resd_1 = glm::vec4(glm::cross(r1, relative_n), 0.0f) * i1;
        glm::vec4 resd_2 = glm::vec4(glm::cross(r2, relative_n), 0.0f) * i2;

        glm::vec3 d2 = glm::cross(glm::vec3(resd_1.x, resd_1.y, resd_1.z), r1);
        glm::vec3 d3 = glm::cross(glm::vec3(resd_2.x, resd_2.y, resd_2.z), r2);
        float denominator = d1 + glm::dot(relative_n, d2 + d3);

        float j = (denominator == 0.0f) ? 0.0f : numerator / denominator;
        if (nbC > 0.0f && j != 0.0f) {
            j /= (float)nbC;
        }


        //Update LINEAR VELOCITY
        glm::vec3 impulse = relative_n * j;
        if (!rb_1->static_RB) rb_1->velocity = rb_1->velocity - impulse * rb_1->inverseOfMass;
        if (!rb_2->static_RB) rb_2->velocity = rb_2->velocity + impulse * rb_2->inverseOfMass;

        //Update ANGULAR VELOCITY
        if (!rb_1->static_RB) {
            glm::vec4 res = glm::vec4(glm::cross(r1, impulse), 0.0f) * i1;
            rb_1->angularSpeed = rb_1->angularSpeed - glm::vec3(res.x, res.y, res.z);
        }
        if (!rb_2->static_RB) {
            glm::vec4 res = glm::vec4(glm::cross(r2, impulse), 0.0f) * i2;
            rb_2->angularSpeed = rb_2->angularSpeed + glm::vec3(res.x, res.y, res.z);
        }


        // Friction
        //t : tangential vector to the collision normal
        glm::vec3 t = relative_vel - (relative_n * d_vn);

        if (compareWithEpsilon(glm::dot(t, t), 0.0f)) {
            return;
        }
        t = glm::normalize(t);

        //Find magnitude of friction to apply
        numerator = -glm::dot(relative_vel, t);
        d1 = invMassSum;

        resd_1 = glm::vec4(glm::cross(r1, t), 0.0f) * i1;
        resd_2 = glm::vec4(glm::cross(r2, t), 0.0f) * i2;

        d2 = glm::cross(glm::vec3(resd_1.x, resd_1.y, resd_1.z), r1);
        d3 = glm::cross(glm::vec3(resd_2.x, resd_2.y, resd_2.z), r2);
        denominator = d1 + glm::dot(t, d2 + d3);

        if (denominator == 0.0f) {
            return;
        }

        float jt = numerator / invMassSum;
        if (nbC > 0 && jt != 0.0f) {
            jt /= (float)nbC;
        }

        if (compareWithEpsilon(jt, 0.0f)) {
            return;
        }

        //Coulomb's law for friction :
        //The magnitude of friction can never be greater than or smaller than j scaled by the Coefficient of Friction, which is the square root of the product of the Coefficient of friction for both the objects
        float friction = sqrtf(rb_1->cineticFriction * rb_2->cineticFriction);
        if (jt > j * friction) {
            jt = j * friction;
        }
        else if (jt < -j * friction) {
            jt = -j * friction;
        }


        //Finally add tangent impulse friction to the velocity of components
        glm::vec3 tangentImpuse = t * jt;
        if (!rb_1->static_RB) rb_1->velocity = rb_1->velocity - tangentImpuse * rb_1->inverseOfMass;
        if (!rb_2->static_RB) rb_2->velocity = rb_2->velocity + tangentImpuse * rb_2->inverseOfMass;


        if (!rb_1->static_RB) {
            glm::vec4 res = glm::vec4(glm::cross(r1, tangentImpuse), 0.0f) * i1;
            rb_1->angularSpeed = rb_1->angularSpeed + glm::vec3(res.x, res.y, res.z);
        }
        if (!rb_2->static_RB) {
            glm::vec4 res = glm::vec4(glm::cross(r2, tangentImpuse), 0.0f) * i2;
            rb_2->angularSpeed = rb_2->angularSpeed - glm::vec3(res.x, res.y, res.z);
        }

    }
}
void RigidBodySystem::linearAndAngular_resolveConstraintVolumeRB_Euler_dynamicfriction(RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res, int nbC, const glm::vec3& posrb1, const glm::vec3& posrb2, const glm::mat4& inverseTensorrb1, const glm::mat4& inverseTensorrb2) {
    //std::cout << "LINEAR AND ANGULAR RESOLVE CONSTRAINT VOLUME RB DYNAMIC FRICTION" << std::endl;

    //We will apply an impulse reaction to perform ejections of both elem
    if (!rb_1->static_RB || !rb_2->static_RB) {
        float invMassSum = rb_1->inverseOfMass + rb_2->inverseOfMass;
        if (invMassSum == 0.0f) return;

        glm::vec3 r1 = p_res->point - posrb1;
        glm::vec3 r2 = p_res->point - posrb2;

        glm::mat4 i1 = inverseTensorrb1;
        glm::mat4 i2 = inverseTensorrb2;


        //The cross product of angular velocityand the relative contact point will give us the magnitude of rotational velocity
        glm::vec3 relative_vel = (rb_1->velocity + glm::cross(rb_2->angularSpeed, r2)) - (rb_1->angularSpeed + glm::cross(rb_2->angularSpeed, r2));

        // Relative collision normal
        glm::vec3 relative_n = glm::normalize(p_res->normal);

        //Magnitude of the relative velocity in the direction of the collision normal
        float d_vn = glm::dot(relative_vel, relative_n);

        // Moving away from each other? Do nothing!
        if (d_vn > 0.0f) {
            return;
        }

        float e = fminf(rb_1->bounce, rb_2->bounce);
        float numerator = (-(1.0f + e) * d_vn);
        float d1 = invMassSum;

        glm::vec4 resd_1 = glm::vec4(glm::cross(r1, relative_n), 0.0f) * i1;
        glm::vec4 resd_2 = glm::vec4(glm::cross(r2, relative_n), 0.0f) * i2;

        glm::vec3 d2 = glm::cross(glm::vec3(resd_1.x, resd_1.y, resd_1.z), r1);
        glm::vec3 d3 = glm::cross(glm::vec3(resd_2.x, resd_2.y, resd_2.z), r2);
        float denominator = d1 + glm::dot(relative_n, d2 + d3);

        float j = (denominator == 0.0f) ? 0.0f : numerator / denominator;
        if (nbC > 0.0f && j != 0.0f) {
            j /= (float)nbC;
        }


        //Update LINEAR VELOCITY
        glm::vec3 impulse = relative_n * j;
        if (!rb_1->static_RB) rb_1->velocity = rb_1->velocity - impulse * rb_1->inverseOfMass;
        if (!rb_2->static_RB) rb_2->velocity = rb_2->velocity + impulse * rb_2->inverseOfMass;

        //Update ANGULAR VELOCITY
        if (!rb_1->static_RB) {
            glm::vec4 res = glm::vec4(glm::cross(r1, impulse), 0.0f) * i1;
            rb_1->angularSpeed = rb_1->angularSpeed - glm::vec3(res.x, res.y, res.z);
        }
        if (!rb_2->static_RB) {
            glm::vec4 res = glm::vec4(glm::cross(r2, impulse), 0.0f) * i2;
            rb_2->angularSpeed = rb_2->angularSpeed + glm::vec3(res.x, res.y, res.z);
        }


        // Friction
        //t : tangential vector to the collision normal
        glm::vec3 t = relative_vel - (relative_n * d_vn);

        if (compareWithEpsilon(glm::dot(t, t), 0.0f)) {
            return;
        }
        t = glm::normalize(t);

        //Find magnitude of friction to apply
        numerator = -glm::dot(relative_vel, t);
        d1 = invMassSum;

        resd_1 = glm::vec4(glm::cross(r1, t), 0.0f) * i1;
        resd_2 = glm::vec4(glm::cross(r2, t), 0.0f) * i2;

        d2 = glm::cross(glm::vec3(resd_1.x, resd_1.y, resd_1.z), r1);
        d3 = glm::cross(glm::vec3(resd_2.x, resd_2.y, resd_2.z), r2);
        denominator = d1 + glm::dot(t, d2 + d3);

        if (denominator == 0.0f) {
            return;
        }

        float jt = numerator / invMassSum;
        if (nbC > 0 && jt != 0.0f) {
            jt /= (float)nbC;
        }

        if (compareWithEpsilon(jt, 0.0f)) {
            return;
        }

     
        glm::vec3 tangentImpuse;
        float sf = sqrtf(rb_1->staticFriction * rb_2->staticFriction);
        float df = sqrtf(rb_1->cineticFriction * rb_1->cineticFriction);
        if (fabsf(jt) < j * sf) {
            tangentImpuse = t * jt;
        }
        else {
            tangentImpuse = t * -j * df;
        }


        if (!rb_1->static_RB) rb_1->velocity = rb_1->velocity - tangentImpuse * rb_1->inverseOfMass;
        if (!rb_2->static_RB) rb_2->velocity = rb_1->velocity + tangentImpuse * rb_2->inverseOfMass;


        if (!rb_1->static_RB) {
            glm::vec4 res = glm::vec4(glm::cross(r1, tangentImpuse), 0.0f) * i1;
            rb_1->angularSpeed = rb_1->angularSpeed - glm::vec3(res.x, res.y, res.z);
        }
        if (!rb_2->static_RB) {
            glm::vec4 res = glm::vec4(glm::cross(r2, tangentImpuse), 0.0f) * i2;
            rb_2->angularSpeed = rb_2->angularSpeed + glm::vec3(res.x, res.y, res.z);
        }

    }
}*/





