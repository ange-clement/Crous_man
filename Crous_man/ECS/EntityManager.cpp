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

#include "../Shaders/BasicGShader.hpp"
#include "../Shaders/BlinnPhongLShader.hpp"
#include "../Shaders/SingleTextureQuadShader.hpp"

#include "../InputManager.hpp"

#include "EntityManager.hpp"

#include "../Components/Mesh.hpp"
#include "../Components/Renderer.hpp"
#include "../Components/PointLight.hpp"
#include "../Components/Camera.hpp"
#include "../Components/Spin.hpp"
#include "../Components/Controllers/FlyingController.hpp"


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
}

void EntityManager::initUtil() {
    new BasicShapeRender();
    new InputManager();
}

void EntityManager::initShaders() {
    new BasicGShader();
    new BlinnPhongLShader();
    new SingleTextureQuadShader();
}

void EntityManager::initSystems() {
    if (SystemIDs::NUMBER >= 16) {
        std::cout << "WARNING TOO MANY COMPONENTS" << std::endl;
    }
    systems.resize(SystemIDs::NUMBER);
    systems[SystemIDs::MeshID] = new MeshSystem();
    systems[SystemIDs::RendererID] = new RendererSystem();
    systems[SystemIDs::PointLightID] = new PointLightSystem();
    systems[SystemIDs::CameraID] = new CameraSystem();
    systems[SystemIDs::SpinID] = new SpinSystem();
    systems[SystemIDs::FlyingControllerID] = new FlyingControllerSystem();
}

void EntityManager::initializeAllSystems() {
    for (size_t i = 0, size = systems.size(); i < size; i++) {
        systems[i]->initializeAll();
    }
}

void EntityManager::updateAllSystems() {
    for (size_t i = 0, size = systems.size(); i < size; i++) {
        systems[i]->updateAll();
    }
}

void EntityManager::updateTransforms() {
    this->entities[0]->updateTransforms();
}


unsigned short EntityManager::getComponentId(SystemIDs system, unsigned short entityID) {
    return systems[system]->getComponentId(entityID);
}

bool EntityManager::hasComponent(SystemIDs system, unsigned short entityID) {
    Bitmap* b = this->systems[system]->requiredComponentsBitmap;
    return this->entities[entityID]->componentsBitmap->combine(b)->equals(b);
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