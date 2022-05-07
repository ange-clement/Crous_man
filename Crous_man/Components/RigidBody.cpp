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
    // 3 - Apply impulses to resolve collisions (except if static)

    RigidBody* rb = getRigidBody(i);
    RigidBody* otherRb;

    Entity* e = EntityManager::instance->entities[entityID];
    Entity* otherE;
    
    for (unsigned int c = 0, size = collisionResults.size(); c < size; c++) {
        //First get the RB of the other member of the collision
        otherE = EntityManager::instance->entities[collisionResults[c]->entityCollidID];
        otherRb = getRigidBodyFromEntityId(collisionResults[c]->entityCollidID);

        //Apply impulse for collision resolution
        unsigned int sizeContact = collisionResults[c]->contactsPts.size();
        if (sizeContact > 0) {
            float invSize = 1.0f / (float)sizeContact;
            for (unsigned int p = 0; p < sizeContact; p++) {
                
                //DEBUG
                std::cout << "collision rigid" << std::endl;
                std::cout << "point : ";
                print(collisionResults[c]->contactsPts[p]->point);
                std::cout << "normal : ";
                print(collisionResults[c]->contactsPts[p]->normal);
                std::cout << "point in local space : ";
                glm::vec3 pointA = e->transform->worldToLocal(collisionResults[c]->contactsPts[p]->point);
                print(pointA);
                glm::vec3 pointB = otherE->transform->worldToLocal(collisionResults[c]->contactsPts[p]->point);
                print(pointB);

                //For particles, we dont make bounces
                
                /*if (rb->type == RBType::VOLUME && otherRb->type == RBType::VOLUME) {
                    resolveConstraintVolumeRB_Euler(rb, otherRb, collisionResults[c]->contactsPts[p], sizeContact);
                }*/
                
                //old fnc
                if (!rb->static_RB) {
                    glm::vec3 centerOfMass = glm::vec3(0.0f);
                    glm::vec3 rA = pointA - centerOfMass;
                    glm::vec3 rB = pointB - centerOfMass;
                    float j = computeAngularFactor(rb, otherRb, collisionResults[c]->contactsPts[p]->normal, rA, rB);
                    rb->addImpulseAtPosition(collisionResults[c]->contactsPts[p]->normal * j / invSize, pointA);
                }
          
                /*
                (new EntityBuilder({ SystemIDs::MeshID, SystemIDs::RendererID }))
                    ->setTranslation(collisionResults[c]->contactsPts[p]->point)
                    ->setRendererDiffuseColor(glm::vec3(0.0, 1.0, 1.0))
                    ->setScale(glm::vec3(0.2, 0.2, 0.2))
                    ->setMeshAsFile("../ressources/Models/suzanne.off", false)
                    ->updateRenderer()
                    ->initializeComponents()
                    ->build();
                */
                //rb->addImpulse(collisionResults[c]->contactsPts[p]->normal * 0.01f / invSize);
            }
        }
        else {
            std::cout << "trigger" << std::endl;
        }
    }
}

void RigidBodySystem::updatePhysics(unsigned short i, unsigned short entityID) {
    //2 - Update position of every rb 
    //3 - Correct sinking using Linear Projection
    //4 - Solve contraint if applicable


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




void RigidBodySystem::initForcesRB(RigidBody* rb) {
    rb->combinedForces              = gravity * rb->mass;
    rb->combinedAngularForces       = glm::vec3(0.0f);
    rb->combinedStaticFriction      = glm::vec3(0.0f);
    rb->combinedCineticFriction     = glm::vec3(0.0f);
}

glm::vec3 RigidBodySystem::update_EulerIntegration(RigidBody* rb, const glm::vec3& currentPos, float deltaTime) {
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

glm::vec3 RigidBodySystem::update_AccurateEulerIntegration(RigidBody* rb, const glm::vec3& currentPos, float deltaTime) {
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
    return currentPos +(velocity * rb->cineticFriction + rb->combinedForces * deltaSquare);
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

//Resolves constraints : VOLUMES (Colliders objects)
void RigidBodySystem::resolveConstraintVolumeRB_Euler(RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res, int nbC){
    //We will apply an impulse reaction to perform ejections of both elem
    if(!rb_1->static_RB || !rb_2->static_RB){

        float invMassSum = rb_1->inverseOfMass + rb_2->inverseOfMass;
    
        
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
        if(!rb_1->static_RB) rb_1->velocity = rb_1->velocity - impulse * rb_1->inverseOfMass;
        if(!rb_2->static_RB) rb_2->velocity = rb_2->velocity + impulse * rb_2->inverseOfMass;

        //Applying friction now
        //t : tangential vector to the collision normal
        glm::vec3 t = relative_vel - (relative_n * d_vn);
        
        if (compareWithEpsilon(glm::dot(t,t), 0.0f)) {
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
        if(!rb_1->static_RB) rb_1->velocity = rb_1->velocity - tangentImpuse * rb_1->inverseOfMass;
        if(!rb_2->static_RB) rb_2->velocity = rb_1->velocity + tangentImpuse * rb_2->inverseOfMass;
    }
}

//Correct new RB position
void RigidBodySystem::positionCorrection(glm::vec3& newPos1, glm::vec3& newPos2, RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res){
    float totalMass = rb_1->inverseOfMass + rb_2->inverseOfMass;
    if (totalMass == 0.0f) { 
        return; 
    }
    float depth = fmaxf(p_res->penetrationDistance - penetrationSlack, 0.0f);
    float scalar = depth / totalMass;
    glm::vec3 correction = p_res->normal * scalar * linearProjectionPercent;

    newPos1 = newPos1 - correction * rb_1->inverseOfMass;
    newPos2 = newPos2 - correction * rb_2->inverseOfMass;
}


glm::mat4 RigidBodySystem::inverseTensor(Collider c, RigidBody* rb){
	if (rb->mass == 0) {
		return glm::mat4(0);
	}

	float ix = 0.0f;
	float iy = 0.0f;
	float iz = 0.0f;
	float iw = 0.0f;

	if (c.type == colliderType::Sphere) {
		float r2 = c.radius * c.radius;
		float fraction = (2.0f / 5.0f);

		ix = r2 * rb->mass * fraction;
		iy = r2 * rb->mass * fraction;
		iz = r2 * rb->mass * fraction;
		iw = 1.0f;
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
		iw = 1.0f;
	}

	return glm::mat4(
		1.0f/ix, 0, 0, 0,
		0, 1.0f/iy, 0, 0,
		0, 0, 1.0f/iz, 0,
		0, 0, 0, 1.0f/iw);
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