#ifndef DESTRUCTIBLE_COMPONENT_HPP
#define DESTRUCTIBLE_COMPONENT_HPP

#include "../ECS/ComponentSystem.hpp"

struct Destructible {
    float destructionAmount = 1.0;
    std::vector<std::string> fragmentMeshFiles;
};

class DestructibleSystem : public virtual ComponentSystem {
public:
    RendererSystem* rendererSystem;
public:
    DestructibleSystem();

    ~DestructibleSystem();

    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void update(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    Destructible* getDestructible(unsigned short i);

    void destroy(unsigned short i);
};

#endif