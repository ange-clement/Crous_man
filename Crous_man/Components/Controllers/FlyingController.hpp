#ifndef FLYING_CONTROLLER_COMPONENT_HPP
#define FLYING_CONTROLLER_COMPONENT_HPP

#include <Crous_man/ECS/ComponentSystem.hpp>

class ColliderSystem;
class EntityPool;
class Cooldown;

struct FlyingController {
    float speed;
    float sensitivity;

    float azimuth;
    float zenith;

    EntityPool* pool;

    Cooldown* rayCastCooldown;
};

class FlyingControllerSystem : public virtual ComponentSystem {
public:
    ColliderSystem* colliderSystem = NULL;
public:
    FlyingControllerSystem();

    ~FlyingControllerSystem();

    virtual void update(unsigned short i, unsigned short entityID);
    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    FlyingController* getFlyingController(unsigned short i);
};

#endif