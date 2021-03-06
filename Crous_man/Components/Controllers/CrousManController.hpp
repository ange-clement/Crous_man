#ifndef CROUS_MAN_CONTROLLER_COMPONENT_HPP
#define CROUS_MAN_CONTROLLER_COMPONENT_HPP

#include <Crous_man/ECS/ComponentSystem.hpp>

class ColliderSystem;
class DestructibleSystem;
class RigidBody;

struct PlayingAudio;

struct CrousManController {
    float maxSpeed;
    float acceleration;
    float sensitivity;

    float azimuth;
    float zenith;

    float maxCameraDistance = 50.0f;
    glm::vec3 initialRotatingPos;

    Entity* rotatingCenterForCamera;
    Entity* cameraTarget;
    Entity* camera;
    Entity* saucisseEntity;
    Entity* laserEntity;

    Cooldown* laserSoundCooldown;
    bool firstLaserSound = true;
    unsigned int laserAudioInd = (unsigned int)-1;

    RigidBody* rb;
};
//ColliderManagerComponentSystem
class CrousManControllerSystem : public virtual ColliderManagerComponentSystem {
public:
    ColliderSystem* colliderSystem = NULL;
    DestructibleSystem* destructibleSystem = NULL;
public:
    CrousManControllerSystem();

    ~CrousManControllerSystem();

    virtual void updateOnCollide(unsigned short i, unsigned short entityID, const std::vector<ColliderResult*> & collisionResults);
    virtual void update(unsigned short i, unsigned short entityID);
    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    CrousManController* getCrousManController(unsigned short i);
};

#endif