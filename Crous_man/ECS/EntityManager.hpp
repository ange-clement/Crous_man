#ifndef ENTITY_MANAGER_HPP
#define ENTITY_MANAGER_HPP

#include "SystemIDs.hpp"

struct Mesh;
struct Renderer;
struct Camera;
struct PointLight;
struct Collider;
struct RigidBody;
struct Destructible;
struct DeleteAfterTime;
struct Spin;
struct FollowObject;
struct FlyingController;
struct SimpleMovementPlayer;
struct CrousManController;

class Entity;
class ComponentSystem;

class EntityManager {
public:
    static EntityManager* instance;

    std::vector<Entity*> entities;
    std::vector<ComponentSystem*> systems;

    std::vector<Mesh> meshComponents;
    std::vector<Renderer> rendererComponents;
    std::vector<Camera> cameraComponents;
    std::vector<PointLight> pointLightComponents;
    std::vector<Collider> colliderComponents;
    std::vector<RigidBody> rigidBodyComponents;
    std::vector<Spin> spinComponents;
    std::vector<Destructible> destructibleComponents;
    std::vector<DeleteAfterTime> deleteAfterTimeComponents;
    std::vector<FollowObject> followObjectComponents;
    std::vector<FlyingController> flyingControllerComponents;
    std::vector<SimpleMovementPlayer> simplePlayerControllerComponents;
    std::vector<CrousManController> crousManControllerComponents;
public:

    EntityManager();
    ~EntityManager();

    void initUtil();
    void initShaders();
    void initSystems();

    void initializeAllSystems();

    void update();
    void updateTransforms();
    void updateCollision();
    void updateOnCollide();
    void updatePhysics();
    void updateAfterPhysics();
    void render();
    void updateAfterRender();

    unsigned short getComponentId(SystemIDs system, unsigned short entityID);
    bool hasComponent(SystemIDs system, unsigned short entityID);
    bool shouldUpdate(unsigned short entityID);

    void addEntity(Entity* entity);
    void reevaluateEntity(Entity* entity);
    void removeEntity(Entity* entity);
};

#endif