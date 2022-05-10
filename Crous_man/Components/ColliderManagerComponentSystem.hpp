#ifndef COMPONENT_SYSTEM_COLLIDER_MANAGER_HPP
#define COMPONENT_SYSTEM_COLLIDER_MANAGER_HPP

#include "../ECS/ComponentSystem.hpp"

class ColliderSystem;

class ColliderManagerComponentSystem : public virtual ComponentSystem {
public :
    ColliderSystem* colliderSystemInstance;
public:
    ColliderManagerComponentSystem();
    virtual ~ColliderManagerComponentSystem();
    virtual void updateOnCollideAll();
};

#endif