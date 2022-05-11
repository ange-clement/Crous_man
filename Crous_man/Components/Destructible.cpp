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
#include "../Util.hpp"

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

#define DESTRUCTIBLE_SYSTEM true

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
        Entity* childFragment = (new EntityBuilder({ SystemIDs::ColliderID, SystemIDs::RigidBodyID, SystemIDs::MeshID, SystemIDs::RendererID, SystemIDs::DeleteAfterTimeID}))
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

float getRandomValue() {
    return ((rand() /(float) RAND_MAX) - .5) * 2.0;
}

void DestructibleSystem::setFragmentParameters(Entity* myself, Destructible* d, Entity* e) {
    if (DESTRUCTIBLE_SYSTEM){
        std::cout << "MYSELF : " << myself->id << std::endl;
        std::cout << "SET FRAG PARAM" << std::endl;
        std::cout << "ID : " << e->id << std::endl;
    }

    e->transform->scaling = d->fragmentScaling;
    e->updateTransforms();

    glm::vec3 centerToChild = glm::vec3(getRandomValue(), getRandomValue(), getRandomValue());
    
    if (glm::dot(centerToChild, centerToChild) > FLT_EPSILON) {
        centerToChild = glm::normalize(centerToChild);
    }
    float explosionAmount = 100.0f;

    if (DESTRUCTIBLE_SYSTEM) std::cout << "IF STATEMENT" << std::endl;

    if (EntityManager::instance->hasComponent(SystemIDs::RigidBodyID, e->id)) {
         if (DESTRUCTIBLE_SYSTEM) std::cout << "IF STATEMENT IN" << std::endl;
         RigidBody* rb = rigidBodySystem->getRigidBodyFromEntityId(e->id);
         if (DESTRUCTIBLE_SYSTEM) std::cout << "RB FROM ENTITY ID : " << rb << std::endl;

        rb->static_RB = false;
        rb->addImpulse(centerToChild * explosionAmount);
        
        if (DESTRUCTIBLE_SYSTEM) std::cout << "rb : " << rb << std::endl;
        unsigned int id = rigidBodySystem->getComponentId(e->id);
        if (DESTRUCTIBLE_SYSTEM) std::cout << "ID : " << id << std::endl;

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

    if (DESTRUCTIBLE_SYSTEM) {
        std::cout << "END" << e->id << std::endl;
    }

    // for (size_t c = 0, size = e->childrens.size(); c < size; c++) {
    //     setFragmentParameters(d, e->childrens[c]);
    // }
}

void DestructibleSystem::destroy(unsigned short i) {
    Entity* entity = EntityManager::instance->entities[this->entityIDs[i]];
    SoundManager::instance->playAt("../ressources/Sounds/explosion.wav", entity->worldTransform->translation);
    

    entity->removeComponent(SystemIDs::DestructibleID);
    if (DESTRUCTIBLE_SYSTEM) {
        std::cout << "FOR" << std::endl;
        std::cout << "ENTITY CHILDRENS : " << entity->childrens.size() << std::endl;
    }
    for (size_t c = 0, size = entity->childrens.size(); c < size; c++) {
        
        entity->childrens[c]->isActive = true;
        setFragmentParameters(entity, getDestructible(i), entity->childrens[c]);
    }
    entity->isActive = false;
}

void DestructibleSystem::destroyAmount(unsigned short i, float amount) {
    getDestructible(i)->destructionAmount -= amount;
    if (getDestructible(i)->destructionAmount < 0) {
        destroy(i);
    }
}