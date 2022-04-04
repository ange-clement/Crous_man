#ifndef SPIN_COMPONENT_HPP
#define SPIN_COMPONENT_HPP

#include "../ECS/ComponentSystem.hpp"

struct Spin {
    float speed;
    float spinAmount;
};

class SpinSystem : public virtual ComponentSystem {
public:
    SpinSystem();

    ~SpinSystem();

    virtual void update(unsigned short i, unsigned short entityID);
    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    Spin* getSpin(unsigned short i);
};

#endif