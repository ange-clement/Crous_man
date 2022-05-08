#ifndef CROUS_MAN_CONTROLLER_COMPONENT_HPP
#define CROUS_MAN_CONTROLLER_COMPONENT_HPP

#include <Crous_man/ECS/ComponentSystem.hpp>

class ColliderSystem;
class RigidBody;

struct CrousManController {
    float maxSpeed;
    float acceleration;
    float sensitivity;

    Entity* meshEntity;
    float azimuth;
    float zenith;
    float lastMoovedAzimuth;
    float lastMoovedZenith;

    Cooldown* laserCooldown;

    RigidBody* rb;
};

class CrousManControllerSystem : public virtual ComponentSystem {
public:
    ColliderSystem* colliderSystem = NULL;
public:
    CrousManControllerSystem();

    ~CrousManControllerSystem();

    virtual void update(unsigned short i, unsigned short entityID);
    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    CrousManController* getCrousManController(unsigned short i);
};

#endif