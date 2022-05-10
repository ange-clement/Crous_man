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
#include "RigidBody.hpp"
#include "Mesh.hpp"
#include "Collider.hpp"

#include "Destructible.hpp"


DestructibleSystem::DestructibleSystem() : ComponentSystem() {
    requiredComponentsBitmap = new Bitmap({ SystemIDs::DestructibleID });
    rigidBodySystem = dynamic_cast<RigidBodySystem*>(EntityManager::instance->systems[SystemIDs::RigidBodyID]);
    meshSystem = dynamic_cast<MeshSystem*>(EntityManager::instance->systems[SystemIDs::MeshID]);
    colliderSystem = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID]);
}

DestructibleSystem::~DestructibleSystem() {

}

void DestructibleSystem::initialize(unsigned short i, unsigned short entityID) {

    Destructible* destructible = getDestructible(i);
    Entity* entity = EntityManager::instance->entities[entityID];

    for (size_t s = 0, size = destructible->fragmentMeshFiles.size(); s < size; s++) {
        Entity* childFragment = (new EntityBuilder({ SystemIDs::ColliderID, SystemIDs::RigidBodyID, SystemIDs::MeshID, SystemIDs::RendererID, SystemIDs::DeleteAfterTimeID }))
            ->setChildOf(entity)
            ->setMeshAsFilePLY(destructible->fragmentMeshFiles[s], destructible->fragmentMeshInvertTriangle[s])
            ->fitAABBColliderToMesh()
            ->setRenderingCollider()
            ->setActive(false)
            ->updateRenderer()
            ->setRendererDiffuseColor(destructible->fragmentColor)
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

void DestructibleSystem::setFragmentParameters(Destructible* d, Entity* e) {
    e->transform->scaling = d->fragmentScaling;
    e->updateTransforms();

    if (EntityManager::instance->hasComponent(SystemIDs::RigidBodyID, e->id)) {
        unsigned int id = rigidBodySystem->getComponentId(e->id);
        RigidBody* rb = rigidBodySystem->getRigidBody(id);
        rb->static_RB = true;

        if (EntityManager::instance->hasComponent(SystemIDs::MeshID, e->id) && EntityManager::instance->hasComponent(SystemIDs::ColliderID, e->id)) {
            unsigned int meshId = meshSystem->getComponentId(e->id);
            Mesh* mesh = meshSystem->getMesh(meshId);
            
	        unsigned short colliderId = colliderSystem->getComponentId(e->id);
	        Collider* collider = colliderSystem->getCollider(colliderId);

            collider->type = colliderType::AABB;
            computeBox(e->worldTransform, mesh->indexed_vertices, collider->center, collider->size);
            colliderSystem->initialize(colliderId, e->id);
        }
    }

    // for (size_t c = 0, size = e->childrens.size(); c < size; c++) {
    //     setFragmentParameters(d, e->childrens[c]);
    // }
}

void DestructibleSystem::destroy(unsigned short i) {
    Entity* entity = EntityManager::instance->entities[this->entityIDs[i]];
    SoundManager::instance->playAt("../ressources/Sounds/explosion.wav", entity->worldTransform->translation);
    
    entity->removeComponent(SystemIDs::DestructibleID);
    for (size_t c = 0, size = entity->childrens.size(); c < size; c++) {
        entity->childrens[c]->isActive = true;
        setFragmentParameters(getDestructible(i), entity->childrens[c]);
    }
    entity->isActive = false;
}

void DestructibleSystem::destroyAmount(unsigned short i, float amount) {
    getDestructible(i)->destructionAmount -= amount;
    if (getDestructible(i)->destructionAmount < 0) {
        destroy(i);
    }
}