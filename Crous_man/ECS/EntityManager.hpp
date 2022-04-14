#ifndef ENTITY_MANAGER_HPP
#define ENTITY_MANAGER_HPP

#include "SystemIDs.hpp"

struct Mesh;
struct Renderer;
struct Camera;
struct PointLight;
struct Spin;
struct FlyingController;

class Entity;
class ComponentSystem;

class EntityManager {
public:
    static EntityManager* instance;

    std::vector<Entity*> entities;
    std::vector<ComponentSystem*> systems;

    std::vector<Mesh> meshComponents;
    std::vector<Renderer> rendererComponents;
    std::vector<PointLight> pointLightComponents;
    std::vector<Camera> cameraComponents;
    std::vector<Spin> spinComponents;
    std::vector<FlyingController> flyingControllerComponents;

public:

    EntityManager();
    ~EntityManager();

    void initUtil();
    void initShaders();
    void initSystems();

    void initializeAllSystems();
    void updateAllSystems();

    void updateTransforms();

    unsigned short getComponentId(SystemIDs system, unsigned short entityID);
    bool hasComponent(SystemIDs system, unsigned short entityID);

    void addEntity(Entity* entity);
};

#endif