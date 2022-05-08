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

    /*RigidBody* rb = getRigidBody(i);
    RigidBody* otherRb;

    Entity* e = EntityManager::instance->entities[entityID];
    Entity* otherE;
    
    for (unsigned int c = 0, size = collisionResults.size(); c < size; c++) {
        //First get the RB of the other member of the collision
        //otherE = EntityManager::instance->entities[collisionResults[c]->entityCollidID];
        //otherRb = getRigidBodyFromEntityId(collisionResults[c]->entityCollidID);

        //Apply impulse for collision resolution
        //unsigned int sizeContact = collisionResults[c]->contactsPts.size();
        //if (sizeContact > 0) {
            //float invSize = 1.0f / (float)sizeContact;
            //for (unsigned int p = 0; p < sizeContact; p++) {
                
                //DEBUG
                //std::cout << "collision rigid" << std::endl;
                //std::cout << "point : ";
                //print(collisionResults[c]->contactsPts[p]->point);
                //std::cout << "normal : ";
                //print(collisionResults[c]->contactsPts[p]->normal);
                //std::cout << "point in local space : ";
                //glm::vec3 pointA = e->transform->worldToLocal(collisionResults[c]->contactsPts[p]->point);
                //print(pointA);
                //glm::vec3 pointB = otherE->transform->worldToLocal(collisionResults[c]->contactsPts[p]->point);
                //print(pointB);

                //For particles, we dont make bounces
                
                //if (rb->type == RBType::VOLUME && otherRb->type == RBType::VOLUME) {
                //    resolveConstraintVolumeRB_Euler(rb, otherRb, collisionResults[c]->contactsPts[p], sizeContact);
                //}
                
                //old fnc
                //if (!rb->static_RB) {
                //    glm::vec3 centerOfMass = glm::vec3(0.0f);
                //    glm::vec3 rA = pointA - centerOfMass;
                //    glm::vec3 rB = pointB - centerOfMass;
                //    float j = computeAngularFactor(rb, otherRb, collisionResults[c]->contactsPts[p]->normal, rA, rB);
                //    rb->addImpulseAtPosition(collisionResults[c]->contactsPts[p]->normal * j / invSize, pointA);
                //}
          
                
                //(new EntityBuilder({ SystemIDs::MeshID, SystemIDs::RendererID }))
                //    ->setTranslation(collisionResults[c]->contactsPts[p]->point)
                //    ->setRendererDiffuseColor(glm::vec3(0.0, 1.0, 1.0))
                //    ->setScale(glm::vec3(0.2, 0.2, 0.2))
                //    ->setMeshAsFile("../ressources/Models/suzanne.off", false)
                //    ->updateRenderer()
                //    ->initializeComponents()
                //    ->build();
         
                //rb->addImpulse(collisionResults[c]->contactsPts[p]->normal * 0.01f / invSize);
            //}
        //}
        //else {
        //    std::cout << "trigger" << std::endl;
        //}
    }*/



    /*=========================================================================================================================*/
    bool with_dynamic_friction = false;
    bool with_rotation = false;
    bool correctPos = false;

    RigidBody* rb = getRigidBody(i);
    //Apply basic gravity
    initForcesRB(rb);
    RigidBody* otherRb;

    Entity* e = EntityManager::instance->entities[entityID];
    Entity* otherE;

    Collider* col = getColliderFromEntityID(entityID);
    Collider* otherCol = 0;

    //We can't perform angular impulse without Collider of the element 
    if (col != 0 || !with_rotation) {
        glm::mat4 inversedTensor_I;
        glm::mat4 inversedTensor_J;

        if(with_rotation) inversedTensor_I = getMatrixInverseTensor(entityID, *col, rb);

        glm::vec3 pos = e->worldTransform->translation;
        glm::vec3 otherpos;

        for (unsigned int c = 0, size = collisionResults.size(); c < size; c++) {
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

            //First get the entity and RB of the other member of the collision
            otherE = EntityManager::instance->entities[collisionResults[c]->entityCollidID];
            otherRb = getRigidBodyFromEntityId(collisionResults[c]->entityCollidID);

            otherpos = otherE->worldTransform->translation;
            
            if (with_rotation) {
                otherCol = getColliderFromEntityID(collisionResults[c]->entityCollidID);
                if (otherCol != 0) {
                    inversedTensor_J = getMatrixInverseTensor(entityID, *otherCol, otherRb);
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


                    if (with_rotation) {
                        if (with_dynamic_friction) {
                            linearAndAngular_resolveConstraintVolumeRB_Euler_dynamicfriction(rb, otherRb, cp, sizeContact, pos, otherpos, inversedTensor_I, inversedTensor_J);
                        }
                        else {
                            linearAndAngular_resolveConstraintVolumeRB_Euler(rb, otherRb, cp, sizeContact, pos, otherpos, inversedTensor_I, inversedTensor_J);
                        }
                    }
                    else {
                        if (with_dynamic_friction) {
                            linear_resolveConstraintVolumeRB_Euler_dynamicfriction(rb, otherRb, cp, sizeContact);
                        }
                        else {
                            linear_resolveConstraintVolumeRB_Euler(rb, otherRb, cp, sizeContact);
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
    /*
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

        glm::vec3 angularDragForces = rb->angularSpeed * -rb->angularDrag;
        glm::vec3 totalAngularForces = rb->combinedAngularForces + angularDragForces;
        glm::vec3 angularAcceleration = totalAngularForces * rb->inverseOfMass;

        rb->velocity += acceleration * deltaTime;
        rb->angularSpeed += angularAcceleration * deltaTime;
        Entity* e = EntityManager::instance->entities[entityID];


        if (!rb->static_RB) {
            // Apply on object
            e->transform->translate(rb->velocity);
            e->worldTransform->translate(rb->velocity);

            float rotationAngle = glm::length(rb->angularSpeed);
            if (rotationAngle != 0.0f) {
                e->transform->rotation.combineRotation(rotationAngle, rb->angularSpeed);
            }
        }
    }

    // Reinitialise forces (free fall object)
    rb->combinedForces = gravity * rb->mass;        // initialised by weight
    rb->combinedAngularForces = glm::vec3(0.0f);
    rb->combinedStaticFriction = glm::vec3(0.0f);
    rb->combinedCineticFriction = glm::vec3(0.0f);*/



    /*=========================================================================================================================*/
    bool with_rotation = false;
    RigidBody* rb = getRigidBody(i);
    float deltaTime = InputManager::instance->deltaTime;

    Entity* e = EntityManager::instance->entities[entityID];


    glm::vec3 currentPos = e->worldTransform->translation;
    //linear update part
    glm::vec3 newPos = linear_update_EulerIntegration(rb, currentPos, deltaTime);
    //glm::vec3 newPos = linear_update_AccurateEulerIntegration(rb, currentPos, deltaTime);
    //glm::vec3 newPos = linear_update_VerletIntegration(rb, currentPos, deltaTime);

    //Rotational update part
    glm::mat4 inversedTensor_I;
    Collider* col = getColliderFromEntityID(entityID);
    //We can't perform angular impulse without Collider of the element 
    if (with_rotation && col != 0) {
        inversedTensor_I = getMatrixInverseTensor(entityID, *col, rb);
        
        rotational_update(rb, deltaTime, getFromMat4(inversedTensor_I));

        e->transform->rotation.combineRotation(degToRad(rb->angularSpeed.x), glm::vec3(1,0,0));
        e->transform->rotation.combineRotation(degToRad(rb->angularSpeed.y), glm::vec3(0,1,0));
        e->transform->rotation.combineRotation(degToRad(rb->angularSpeed.z), glm::vec3(0,0,1));
    }

    // Apply on object
    e->transform->translate(newPos - currentPos);
    e->worldTransform->translate(newPos - currentPos);
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

TensorMatrix::TensorMatrix() {
    computed = false;
    mat = glm::mat4(1.0f);
}
TensorMatrix::~TensorMatrix() {}


void RigidBodySystem::clearMatrixTensorStructure(unsigned short id) {
    tensorMatrix[id].computed = false;
}
glm::mat4 RigidBodySystem::getMatrixInverseTensor(unsigned short entityID, Collider c, RigidBody* rb) {
    if (tensorMatrix.size() < entityID) {
        // If tensorMatrix is too small, extend it
        tensorMatrix.resize(entityID + 1, TensorMatrix());
    }
    TensorMatrix t = tensorMatrix[entityID];
    if (!t.computed) {
        t.computed = true;
        t.mat = inverseTensorComputation(c,rb);
        return t.mat;
    }
    else {
        return t.mat;
    }
}


void RigidBodySystem::initForcesRB(RigidBody* rb) {
    rb->combinedForces = gravity * rb->mass;
    rb->combinedAngularForces = glm::vec3(0.0f);
    rb->combinedStaticFriction = glm::vec3(0.0f);
    rb->combinedCineticFriction = glm::vec3(0.0f);
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
glm::vec3 RigidBodySystem::linear_update_EulerIntegration(RigidBody* rb, const glm::vec3& currentPos, float deltaTime) {
    rb->oldPosition = currentPos;
    glm::vec3 acceleration = rb->combinedForces * rb->inverseOfMass;
    rb->velocity = rb->velocity * rb->cineticFriction + acceleration * deltaTime;

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

    return currentPos + rb->velocity * deltaTime;
}
glm::vec3 RigidBodySystem::linear_update_AccurateEulerIntegration(RigidBody* rb, const glm::vec3& currentPos, float deltaTime) {
    rb->oldPosition = currentPos;
    glm::vec3 acceleration = rb->combinedForces * rb->inverseOfMass;
    glm::vec3 oldVelocity = rb->velocity;
    rb->velocity = rb->velocity * rb->cineticFriction + acceleration * deltaTime;

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
glm::vec3 RigidBodySystem::linear_update_VerletIntegration(RigidBody* rb, const glm::vec3& currentPos, float deltaTime) {
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
    return currentPos + (velocity * rb->cineticFriction + rb->combinedForces * deltaSquare);
}

//Resolves constraints : VOLUMES (Colliders objects)
void RigidBodySystem::linear_resolveConstraintVolumeRB_Euler(RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res, int nbC) {
    //We will apply an impulse reaction to perform ejections of both elem
    if (!rb_1->static_RB || !rb_2->static_RB) {
        float invMassSum = rb_1->inverseOfMass + rb_2->inverseOfMass;
        if (invMassSum == 0.0f) return;

        glm::vec3 relative_vel = rb_2->velocity - rb_1->velocity;
        glm::vec3 relative_n = glm::normalize(p_res->normal);

        //Magnitude of the relative velocity in the direction of the collision normal
        float d_vn = glm::dot(relative_vel, relative_n);
        //Moving away from each other
        if (d_vn > 0.0f) return;

        float e = fminf(rb_1->bounce, rb_2->bounce);
        float numerator = (-(1.0f + e) * d_vn);

        //Magnitude of our impulse
        float j = numerator / invMassSum;

        if (nbC > 0 && j != 0.0f) {
            j /= (float)nbC;
        }

        glm::vec3 impulse = relative_n * j;
        if (!rb_1->static_RB) rb_1->velocity = rb_1->velocity - impulse * rb_1->inverseOfMass;
        if (!rb_2->static_RB) rb_2->velocity = rb_2->velocity + impulse * rb_2->inverseOfMass;

        //Applying friction now
        //t : tangential vector to the collision normal
        glm::vec3 t = relative_vel - (relative_n * d_vn);

        if (compareWithEpsilon(glm::dot(t, t), 0.0f)) {
            return;
        }
        t = glm::normalize(t);

        //Find magnitude of friction to apply
        numerator = -glm::dot(relative_vel, t);
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
        if (!rb_2->static_RB) rb_2->velocity = rb_1->velocity + tangentImpuse * rb_2->inverseOfMass;
    }
}
void RigidBodySystem::linear_resolveConstraintVolumeRB_Euler_dynamicfriction(RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res, int nbC) {
    //We will apply an impulse reaction to perform ejections of both elem
    if (!rb_1->static_RB || !rb_2->static_RB) {
        float invMassSum = rb_1->inverseOfMass + rb_2->inverseOfMass;
        if (invMassSum == 0.0f) return;

        glm::vec3 relative_vel = rb_2->velocity - rb_1->velocity;
        glm::vec3 relative_n = glm::normalize(p_res->normal);

        //Magnitude of the relative velocity in the direction of the collision normal
        float d_vn = glm::dot(relative_vel, relative_n);
        //Moving away from each other
        if (d_vn > 0.0f) return;

        float e = fminf(rb_1->bounce, rb_2->bounce);
        float numerator = (-(1.0f + e) * d_vn);

        //Magnitude of our impulse
        float j = numerator / invMassSum;

        if (nbC > 0 && j != 0.0f) {
            j /= (float)nbC;
        }

        glm::vec3 impulse = relative_n * j;
        if (!rb_1->static_RB) rb_1->velocity = rb_1->velocity - impulse * rb_1->inverseOfMass;
        if (!rb_2->static_RB) rb_2->velocity = rb_2->velocity + impulse * rb_2->inverseOfMass;

        //Applying friction now
        //t : tangential vector to the collision normal
        glm::vec3 t = relative_vel - (relative_n * d_vn);

        if (compareWithEpsilon(glm::dot(t, t), 0.0f)) {
            return;
        }
        t = glm::normalize(t);

        //Find magnitude of friction to apply
        numerator = -glm::dot(relative_vel, t);
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
    }
}

//Correct new RB position
void RigidBodySystem::positionCorrection(glm::vec3& newPos1, glm::vec3& newPos2, RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res) {
    float totalMass = rb_1->inverseOfMass + rb_2->inverseOfMass;
    if (totalMass == 0.0f) {
        return;
    }
    float depth = fmaxf(p_res->penetrationDistance - penetrationSlack, 0.0f);
    float scalar = depth / totalMass;
    glm::vec3 correction = p_res->normal * scalar * linearProjectionPercent;

    newPos1 = newPos1 - correction * rb_1->inverseOfMass;
    newPos2 = newPos2 + correction * rb_2->inverseOfMass;
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
        float r2 = c.radius * c.radius;
        float fraction = (2.0f / 5.0f);

        ix = r2 * rb->mass * fraction;
        iy = r2 * rb->mass * fraction;
        iz = r2 * rb->mass * fraction;
        //iw = 1.0f;
    }
    else if (c.type == colliderType::AABB || c.type == colliderType::OBB) {
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

void RigidBodySystem::rotational_update(RigidBody* rb, float deltaTime, const glm::mat3& inverseTensor) {
    glm::vec3 angAccel = rb->combinedAngularForces * inverseTensor;
    rb->angularSpeed = rb->angularSpeed + angAccel * deltaTime;
    rb->angularSpeed = rb->angularSpeed * rb->angularDrag;

    //if the speed is to slow, no more rotation performed
    if (fabsf(rb->angularSpeed.x) < 0.001f) {
        rb->angularSpeed.x = 0.0f;
    }
    if (fabsf(rb->angularSpeed.y) < 0.001f) {
        rb->angularSpeed.y = 0.0f;
    }
    if (fabsf(rb->angularSpeed.z) < 0.001f) {
        rb->angularSpeed.z = 0.0f;
    }

    rb->angularSpeed *= deltaTime;

    //  how to use it next :
    //  Entity* e = EntityManager::instance->entities[entityID];;
    //  e->transform->rotation.combineRotation(degToRad(rb->angularSpeed.x), glm::vec3(1,0,0));
    //  e->transform->rotation.combineRotation(degToRad(rb->angularSpeed.y), glm::vec3(0,1,0));
    //  e->transform->rotation.combineRotation(degToRad(rb->angularSpeed.z), glm::vec3(0,0,1));
}


// TOTAL VELOCITY : (linear plus angular) by adding the rotational velocity to the 
//  Linear Velocity of the rigidbody at the center of mass. We also need to find the torque from
//  the point of impactand collision normal divided by the inertia tensor.Knowing this, we can
//  find the final equation for j
void RigidBodySystem::linearAndAngular_resolveConstraintVolumeRB_Euler(RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res, int nbC, const glm::vec3& posrb1, const glm::vec3& posrb2, const glm::mat4& inverseTensorrb1, const glm::mat4& inverseTensorrb2) {
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
}

void RigidBodySystem::linearAndAngular_resolveConstraintVolumeRB_Euler_dynamicfriction(RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res, int nbC, const glm::vec3& posrb1, const glm::vec3& posrb2, const glm::mat4& inverseTensorrb1, const glm::mat4& inverseTensorrb2) {
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
