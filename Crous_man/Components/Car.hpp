#ifndef CAR_COMPONENT_HPP
#define CAR_COMPONENT_HPP

#include "../ECS/ComponentSystem.hpp"

struct Car {
    unsigned short gowingToward = 0;

    Entity* carPathEntity;
};

class CarSystem : public virtual ComponentSystem {
public:
    CarSystem();

    ~CarSystem();

    virtual void update(unsigned short i, unsigned short entityID);
    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    Car* getCar(unsigned short i);
};

#endif