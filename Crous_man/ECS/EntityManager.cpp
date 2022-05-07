#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "SystemIDs.hpp"
#include "Bitmap.hpp"

#include "../Shaders/GShaders/BasicGShader.hpp"
#include "../Shaders/GShaders/TextureGShader.hpp"
#include "../Shaders/EShaders/DepthMeshEShader.hpp"
#include "../Shaders/EShaders/ShadowQuadEShader.hpp"
#include "../Shaders/LShaders/BlinnPhongLShader.hpp"
#include "../Shaders/LShaders/BlinnPhongShadowLShader.hpp"
#include "../Shaders/PEShaders/SingleTextureQuadShader.hpp"

#include "../InputManager.hpp"
#include "../SoundManager.hpp"

#include "EntityManager.hpp"

#include "../Components/Mesh.hpp"
#include "../Components/Renderer.hpp"
#include "../Components/Camera.hpp"
#include "../Components/Collider.hpp"
#include "../Components/RigidBody.hpp"
#include "../Components/PointLight.hpp"
#include "../Components/Spin.hpp"
#include "../Components/Destructible.hpp"
#include "../Components/Controllers/FlyingController.hpp"
#include "../Components/Controllers/SimpleMovementPlayer.hpp"


#include "ComponentSystem.hpp"
#include "Entity.hpp"
#include "../Transform.hpp"
#include <common/basicShapeRender.hpp>

EntityManager* EntityManager::instance = NULL;

EntityManager::EntityManager() {
    if (EntityManager::instance == NULL) {
        EntityManager::instance = this;
        Entity* root = new Entity();
        root->id = 0;
        entities.push_back(root);
        initUtil();
        initShaders();
        initSystems();
    } else {
        std::cerr << "Error : cannot instanciate two EntityManager" << std::endl;
    }
}

EntityManager::~EntityManager() {
    entities.clear();
    spinComponents.clear();
    EntityManager::instance = NULL;

    delete SoundManager::instance;
}

void EntityManager::initUtil() {
    new BasicShapeRender();
    new InputManager();
    new SoundManager();
}

void EntityManager::initShaders() {
    new BasicGShader();
    new TextureGShader();
    new DepthMeshEShader();
    new ShadowQuadEShader();
    new BlinnPhongShadowLShader();
    new BlinnPhongLShader();
    new SingleTextureQuadShader();
    new TextureGShader();
    new SphereColliderShader();
    new BoxColliderShader();
}

void EntityManager::initSystems() {

    if (SystemIDs::NUMBER >= 16) {
        std::cout << "WARNING TOO MANY COMPONENTS" << std::endl;
    }
    systems.resize(SystemIDs::NUMBER);

    systems[SystemIDs::MeshID] =                    new MeshSystem();
    systems[SystemIDs::RendererID] =                new RendererSystem();
    systems[SystemIDs::PointLightID] =              new PointLightSystem();
    systems[SystemIDs::ColliderID] =                new ColliderSystem();
    systems[SystemIDs::RigidBodyID] =               new RigidBodySystem();
    systems[SystemIDs::CameraID] =                  new CameraSystem();
    systems[SystemIDs::SpinID] =                    new SpinSystem();
    systems[SystemIDs::DestructibleID] =            new DestructibleSystem();
    systems[SystemIDs::FlyingControllerID] =        new FlyingControllerSystem();
    systems[SystemIDs::SimplePlayerControllerID] =  new SimpleMovementPlayerSystem();
}

void EntityManager::initializeAllSystems() {
    for (size_t i = 0, size = systems.size(); i < size; i++) {
        std::cout << "initialize ALL "<< i << std::endl;
        systems[i]->initializeAll();
    }
}




void EntityManager::update() {
    for (size_t i = 0, size = systems.size(); i < size; i++) {
        systems[i]->updateAll();
    }
}
void EntityManager::updateTransforms() {
    this->entities[0]->updateTransforms();
}
void EntityManager::updateCollision() {
    for (size_t i = 0, size = systems.size(); i < size; i++) {
        systems[i]->updateCollisionAll();
    }
}
void EntityManager::updateOnCollide() {
    for (size_t i = 0, size = systems.size(); i < size; i++) {
        systems[i]->updateOnCollideAll();
    }
}
void EntityManager::updatePhysics() {
    for (size_t i = 0, size = systems.size(); i < size; i++) {
        systems[i]->updatePhysicsAll();
    }
}
void EntityManager::updateAfterPhysics() {
    for (size_t i = 0, size = systems.size(); i < size; i++) {
        systems[i]->updateAfterPhysicsAll();
    }
}
void EntityManager::render() {
    for (size_t i = 0, size = systems.size(); i < size; i++) {
        systems[i]->renderAll();
    }
}
void EntityManager::updateAfterRender() {
    for (size_t i = 0, size = systems.size(); i < size; i++) {
        systems[i]->updateAfterRenderAll();
    }
}


unsigned short EntityManager::getComponentId(SystemIDs system, unsigned short entityID) {
    return systems[system]->getComponentId(entityID);
}

bool EntityManager::hasComponent(SystemIDs system, unsigned short entityID) {
    Bitmap* b = this->systems[system]->requiredComponentsBitmap;
    return this->entities[entityID]->componentsBitmap->combine(b)->equals(b);
}

bool EntityManager::shouldUpdate(unsigned short entityID) {
    return entityID != (unsigned short)-1 && entities[entityID]->isActive;
}



void EntityManager::addEntity(Entity* entity) {
    unsigned short id = entities.size();

    Bitmap* entityBitmap;
    Bitmap* systemBitmap;

    for (ComponentSystem* system : systems) {
        entityBitmap = entity->componentsBitmap;
        systemBitmap = system->requiredComponentsBitmap;
        if (entityBitmap->combine(systemBitmap)->equals(systemBitmap)) {
            system->addEntity(id);
        }
    }

    entity->id = id;
    entities.push_back(entity);
    entities[0]->addChildren(entity);
}

void EntityManager::reevaluateEntity(Entity* entity) {
    Bitmap* entityBitmap;
    Bitmap* systemBitmap;

    for (ComponentSystem* system : systems) {
        entityBitmap = entity->componentsBitmap;
        systemBitmap = system->requiredComponentsBitmap;
        if (entityBitmap->combine(systemBitmap)->equals(systemBitmap)) {
            if (!system->containsEntity(entity->id)) {
                system->addEntity(entity->id);
                system->initialize(system->entityIDs.size() - 1, entity->id);
            }
        }
        else if (system->containsEntity(entity->id)) {
            system->removeEntity(entity->id);
        }
    }
}

void EntityManager::removeEntity(Entity* entity) {
    Bitmap* entityBitmap;
    Bitmap* systemBitmap;

    for (ComponentSystem* system : systems) {
        entityBitmap = entity->componentsBitmap;
        systemBitmap = system->requiredComponentsBitmap;
        if (entityBitmap->combine(systemBitmap)->equals(systemBitmap)) {
            system->removeEntity(entity->id);
        }
    }

    entity->removeAllChildrens();
}