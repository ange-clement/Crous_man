#ifndef ROGIDBODY_COMPONENT_HPP
#define ROGIDBODY_COMPONENT_HPP

#include "../ECS/ComponentSystem.hpp"

struct RigidBody {
    glm::vec3 velocity = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 angularSpeed = glm::vec3(0.0f, 0.0f, 0.0f);

    float mass = 1.0f;
    float inverseOfMass = 1.0f;

    float staticFriction = 1.0f;
    float cineticFriction = .5f;

    glm::vec3 combinedForces = glm::vec3(0.0, 0.0, 0.0);
    glm::vec3 combinedAngularForces = glm::vec3(0.0, 0.0, 0.0);
    glm::vec3 combinedStaticFriction  = glm::vec3(0.0, 0.0, 0.0);
    glm::vec3 combinedCineticFriction = glm::vec3(0.0, 0.0, 0.0);

    float drag = 0.5f;
    float angularDrag = 0.2f;

    void setMass(float mass);

    void addForce(glm::vec3 force);
    void addAcceleration(glm::vec3 acc);
    void addVelocity(glm::vec3 vel);
    void addImpulse(glm::vec3 impulse);
    
    void addAngularForce(glm::vec3 force);
    void addAngularAcceleration(glm::vec3 acc);
    void addAngularVelocity(glm::vec3 vel);
    void addAngularImpulse(glm::vec3 impulse);

    void addForceAtPosition(glm::vec3 force, glm::vec3 pos);
    void addAccelerationAtPosition(glm::vec3 acc, glm::vec3 pos);
    void addVelocityAtPosition(glm::vec3 cel, glm::vec3 pos);
    void addImpulseAtPosition(glm::vec3 impulse, glm::vec3 pos);
};

class RigidBodySystem : public virtual ComponentSystem {
public:
    glm::vec3 gravity = glm::vec3(0.0f, -9.81f, 0.0f);
    glm::vec3 gravityDirection = glm::vec3(0.0f, -1.0f, 0.0f);

    std::vector<unsigned short> entityIDsToIndex;
public:
    RigidBodySystem();

    ~RigidBodySystem();

    void setGravity(glm::vec3 gravity);

    virtual void updateOnCollide(unsigned short i, unsigned short entityID, const std::vector<ColliderResult*>& collisionResults);
    virtual void updatePhysics(unsigned short i, unsigned short entityID);
    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    RigidBody* getRigidBody(unsigned short i);
    RigidBody* getRigidBodyFromEntityId(unsigned short entityID);
};

#endif