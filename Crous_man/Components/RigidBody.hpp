#ifndef ROGIDBODY_COMPONENT_HPP
#define ROGIDBODY_COMPONENT_HPP

#include "../ECS/ComponentSystem.hpp"

struct RigidBody {
    glm::vec3 speed = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 acceleration = glm::vec3(0.0f, 0.0f, 0.0f);
    float mass = 1.0f;
};

class RigidBodySystem : public virtual ComponentSystem {
public:
    glm::vec3 gravity = glm::vec3(0.0f, -9.81f, 0.0f);
public:
    RigidBodySystem();

    ~RigidBodySystem();

    virtual void updateOnCollide(unsigned short i, unsigned short entityID, const std::vector<ColliderResult*>& collisionResults);
    virtual void updatePhysics(unsigned short i, unsigned short entityID);
    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    RigidBody* getRigidBody(unsigned short i);
};

#endif