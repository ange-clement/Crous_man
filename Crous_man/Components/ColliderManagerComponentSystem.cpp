#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "../InputManager.hpp"
#include "../ECS/EntityManager.hpp"
#include "../ECS/Entity.hpp"

#include "Collider.hpp"

#include "ColliderManagerComponentSystem.hpp"



ColliderManagerComponentSystem::ColliderManagerComponentSystem() {
    colliderSystemInstance = NULL;
}

ColliderManagerComponentSystem::~ColliderManagerComponentSystem() {

}

void ColliderManagerComponentSystem::updateOnCollideAll() {
    //std::cout << " =========== UPDATE COLLID ALL  =========== " << std::endl;
    //std::cout << " ENTITIES : " << entityIDs.size() << std::endl;

    if (colliderSystemInstance == NULL)
        colliderSystemInstance = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID]);

    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        if (!EntityManager::instance->shouldUpdate(entityIDs[i]))
            continue;
        if (!EntityManager::instance->hasComponent(SystemIDs::ColliderID, entityIDs[i]))
            continue;

        //std::cout << " -> ENTITY ID with collider : " << entityIDs[i] << std::endl;

        std::vector<ColliderResult*> collisionResults;
        std::vector<ColliderResult*> collisionMapResult = colliderSystemInstance->getResultOf(entityIDs[i]);

        for (unsigned int c = 0, size = collisionMapResult.size(); c < size; c++) {
            if (collisionMapResult[c] != NULL && collisionMapResult[c]->isInCollision && collisionMapResult[c]->entityCollidID > entityIDs[i]) {
                collisionResults.push_back(collisionMapResult[c]);
            }
        }

        if (collisionResults.size() == 0) {
            continue;
        }

        //std::cout << " <=> COLLISION ON COLIDE : " << entityIDs[i] << std::endl;
        this->updateOnCollide(i, entityIDs[i], collisionResults);
    }
}