#ifndef ENTITY_MANAGER_HPP
#define ENTITY_MANAGER_HPP

#include "SystemIDs.hpp"

struct Mesh;
struct Renderer;
struct Camera;
struct PointLight;
struct Spin;
struct Destructible;
struct FlyingController;
struct Collider;

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
    std::vector<Destructible> destructibleComponents;
    std::vector<FlyingController> flyingControllerComponents;
    std::vector<Collider> colliderComponents;

public:

    EntityManager();
    ~EntityManager();

    void initUtil();
    void initShaders();
    void initSystems();

    void initializeAllSystems();

    void update();
    void updateTransforms();
    void updatePhysics();
    void updateAfterPhysics();
    void render();
    void updateAfterRender();

    unsigned short getComponentId(SystemIDs system, unsigned short entityID);
    bool hasComponent(SystemIDs system, unsigned short entityID);

    void addEntity(Entity* entity);
};

#endif