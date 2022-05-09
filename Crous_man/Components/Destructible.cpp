#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "../Transform.hpp"
#include "../SoundManager.hpp"

#include "../InputManager.hpp"
#include "../ECS/EntityManager.hpp"
#include "../ECS/Bitmap.hpp"
#include "../ECS/Entity.hpp"
#include "../ECS/EntityBuilder.hpp"

#include "Renderer.hpp"

#include "Destructible.hpp"

DestructibleSystem::DestructibleSystem() : ComponentSystem() {
    requiredComponentsBitmap = new Bitmap({ SystemIDs::DestructibleID });
}

DestructibleSystem::~DestructibleSystem() {
    rendererSystem = dynamic_cast<RendererSystem*>(EntityManager::instance->systems[SystemIDs::RendererID]);
}

void DestructibleSystem::initialize(unsigned short i, unsigned short entityID) {

    Destructible* destructible = getDestructible(i);
    Entity* entity = EntityManager::instance->entities[entityID];

    for (size_t s = 0, size = destructible->fragmentMeshFiles.size(); s < size; s++) {
        Entity* childFragment = (new EntityBuilder({ SystemIDs::ColliderID, SystemIDs::RigidBodyID, SystemIDs::MeshID, SystemIDs::RendererID }))
            ->setChildOf(entity)
            ->setMeshAsFilePLY(destructible->fragmentMeshFiles[s], destructible->fragmentMeshInvertTriangle[s])
            ->fitAABBColliderToMesh()
            ->setRenderingCollider()
            ->setActive(false)
            ->updateRenderer()
            ->setRendererDiffuseColor(glm::vec3(1.0, 1.0, 1.0))
            ->initializeComponents()
            ->build();
    }
}

void DestructibleSystem::update(unsigned short i, unsigned short entityID) {
    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_K) == GLFW_PRESS) {
        SoundManager::instance->play("../ressources/Sounds/SKULL_TRUMPET.ogg");
        destroy(i);
    }
}

void DestructibleSystem::addEntityComponent() {
    EntityManager::instance->destructibleComponents.push_back(Destructible());
}

Destructible* DestructibleSystem::getDestructible(unsigned short i) {
    return &EntityManager::instance->destructibleComponents[i];
}

void DestructibleSystem::destroy(unsigned short i) {
    Entity* entity = EntityManager::instance->entities[this->entityIDs[i]];
    SoundManager::instance->playAt("../ressources/Sounds/explosion.wav", entity->worldTransform->translation);
    
    entity->removeComponent(SystemIDs::DestructibleID);
    entity->setActiveRecursive(true);
    entity->isActive = false;

    for (size_t c = 0, size = entity->childrens.size(); c < size; c++) {
        //Testing
        entity->childrens[c]->transform->translation = glm::vec3(0.3, 0.3, 0.3) * glm::vec3(rand() / (float) RAND_MAX * 2.0 - 1.0, rand() / (float)RAND_MAX * 2.0 - 1.0, rand() / (float)RAND_MAX * 2.0 - 1.0);
    }
}

void DestructibleSystem::destroyAmount(unsigned short i, float amount) {
    getDestructible(i)->destructionAmount -= amount;
    if (getDestructible(i)->destructionAmount < 0) {
        destroy(i);
    }
}