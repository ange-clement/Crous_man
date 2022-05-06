#ifndef ROGIDBODY_COMPONENT_HPP
#define ROGIDBODY_COMPONENT_HPP

#include "../ECS/ComponentSystem.hpp"


enum RBType {
    //A particles has a mass but no volume (no rotation forces)
    PARTICLES,

};

//A rigid body type of element is an element that cant be streachable during time
//got the same shape over time, and also same volume
struct RigidBody {
    RBType type;

    glm::vec3 speed         = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 acceleration  = glm::vec3(0.0f, 0.0f, 0.0f);
    float mass              = 1.0f;


    glm::vec3 forces;
    //More for partciles type of RB
    //Coefficient of Restitution
    float bounce            = .7f;       //represents how much energy is kept when a smt bounces off a surface
    glm::vec3 oldPosition;
};


class RigidBodySystem : public virtual ComponentSystem {
public:
    glm::vec3 gravity = glm::vec3(0.0f, -9.81f, 0.0f);
    float friction = 0.95f;

public:
    RigidBodySystem();

    ~RigidBodySystem();

    virtual void updateOnCollide(unsigned short i, unsigned short entityID, const std::vector<ColliderResult*>& collisionResults);
    virtual void updatePhysics(unsigned short i, unsigned short entityID);
    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    RigidBody* getRigidBody(unsigned short i);


    //Particles treatment
    void applyForcesParticlesRB(RigidBody* rb);
    glm::vec3 updateParticlesRB_EulerIntegration(RigidBody* rb, const glm::vec3& currentPos, float deltaTime);
    glm::vec3 updateParticlesRB_VerletIntegration(RigidBody* rb, const glm::vec3& currentPos, float deltaTime);
    glm::vec3 resolveConstraintParticles(ColliderResult* res, RigidBody* rb, const glm::vec3& currentPos);

};



#endif