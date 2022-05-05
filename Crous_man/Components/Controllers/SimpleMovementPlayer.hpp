#ifndef PLAYER_CONTROLLER_COMPONENT_HPP
#define PLAYER_CONTROLLER_COMPONENT_HPP

#include <Crous_man/ECS/ComponentSystem.hpp>

struct SimpleMovementPlayer {
    float speed;
};

class SimpleMovementPlayerSystem : public virtual ComponentSystem {
public:
    SimpleMovementPlayerSystem();
    ~SimpleMovementPlayerSystem();

    virtual void update(unsigned short i, unsigned short entityID);
    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void updateOnCollide(unsigned short i, unsigned short entityID, const std::vector<ColliderResult*>& collisionResults);
    virtual void addEntityComponent();

    SimpleMovementPlayer* getSimpleMovementController(unsigned short i);
};

#endif