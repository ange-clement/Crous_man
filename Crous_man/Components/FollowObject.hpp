#ifndef FOLLOW_OBJECT_COMPONENT_HPP
#define FOLLOW_OBJECT_COMPONENT_HPP

#include "../ECS/ComponentSystem.hpp"

class Entity;

struct FollowObject {
    float interpolationAmount = 0.1f;

    Entity* entityToFollow = NULL;
};

class FollowObjectSystem : public virtual ComponentSystem {
public:
    FollowObjectSystem();
    ~FollowObjectSystem();

    virtual void updateAfterPhysics(unsigned short i, unsigned short entityID);
    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    FollowObject* getFollowObject(unsigned short i);
};

#endif