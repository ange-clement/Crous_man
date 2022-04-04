#ifndef FLYING_CONTROLLER_HPP
#define FLYING_CONTROLLER_HPP

#include <Crous_man/ECS/ComponentSystem.hpp>

struct FlyingController {
    float speed;
    float sensitivity;

    float azimuth;
    float zenith;
};

class FlyingControllerSystem : public virtual ComponentSystem {
public:
    FlyingControllerSystem();

    ~FlyingControllerSystem();

    virtual void update(unsigned short i, unsigned short entityID);
    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    FlyingController* getFlyingController(unsigned short i);
};

#endif