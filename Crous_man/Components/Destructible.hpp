#ifndef DESTRUCTIBLE_COMPONENT_HPP
#define DESTRUCTIBLE_COMPONENT_HPP

#include "../ECS/ComponentSystem.hpp"

class RendererSystem;
class RigidBodySystem;
class MeshSystem;
class ColliderSystem;
class Entity;

struct Destructible {
    float destructionAmount = 1.0;
    std::vector<std::string> fragmentMeshFiles;
    std::vector<bool> fragmentMeshInvertTriangle;
    glm::vec3 fragmentScaling = glm::vec3(1.0f);
    glm::vec3 fragmentColor = glm::vec3(1.0f, 1.0f, 1.0f);
};

class DestructibleSystem : public virtual ComponentSystem {
public:
    RigidBodySystem* rigidBodySystem;
    MeshSystem* meshSystem;
    ColliderSystem* colliderSystem;
public:
    DestructibleSystem();

    ~DestructibleSystem();

    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void update(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    Destructible* getDestructible(unsigned short i);

    void destroy(unsigned short i);
    void destroyAmount(unsigned short i, float amount);


    void setFragmentParameters(Entity* myself, Destructible* d, Entity* e);
};

#endif