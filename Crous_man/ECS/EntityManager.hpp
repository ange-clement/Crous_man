#ifndef ENTITY_MANAGER_HPP
#define ENTITY_MANAGER_HPP

#include "SystemIDs.hpp"

struct Spin;

class Entity;
class ComponentSystem;

class EntityManager {
public:
    static EntityManager* instance;

    std::vector<Entity*> entities;
    std::vector<ComponentSystem*> systems;

    std::vector<Spin> spinComponents;

public:

    EntityManager();
    ~EntityManager();

    void initSystems();

    void initializeAllSystems();
    void updateAllSystems();

    void updateTransforms();

    unsigned short getComponentId(SystemIDs system, unsigned short entityID);
    bool hasComponent(SystemIDs system, unsigned short entityID);

    void addEntity(Entity* entity);
};

#endif