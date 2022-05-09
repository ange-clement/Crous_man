#ifndef DESTRUCTIBLE_COMPONENT_HPP
#define DESTRUCTIBLE_COMPONENT_HPP

#include "../ECS/ComponentSystem.hpp"

class RendererSystem;

struct Destructible {
    float destructionAmount = 1.0;
    std::vector<std::string> fragmentMeshFiles;
    std::vector<bool> fragmentMeshInvertTriangle;
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
    void destroyAmount(unsigned short i, float amount);
};

#endif