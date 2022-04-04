#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "../ECS/EntityManager.hpp"
#include "../ECS/Entity.hpp"
#include "../ECS/Bitmap.hpp"

#include "../Transform.hpp"

#include "../Components/Mesh.hpp"
#include "../Components/Renderer.hpp"

#include "ECS_test.hpp"

void createSceneECS() {
   
    Entity* test2 = new Entity({SystemIDs::SpinID});
    EntityManager::instance->addEntity(test2);


    Entity* test = new Entity({SystemIDs::MeshID, SystemIDs::RendererID});
    EntityManager::instance->addEntity(test);
    test2->addChildren(test);
    test->transform->translation = glm::vec3(3.0, 0.0, 0.0);
    unsigned short testMeshID = EntityManager::instance->getComponentId(SystemIDs::MeshID, test->id);
    EntityManager::instance->meshComponents[testMeshID].loadFromFile("../ressources/suzanne.off", false);
    unsigned short testRendererID = EntityManager::instance->getComponentId(SystemIDs::RendererID, test->id);
    dynamic_cast<RendererSystem*>(EntityManager::instance->systems[SystemIDs::RendererID])->initBuffers(testRendererID, test->id);


    Entity* cameraEntity = new Entity({SystemIDs::CameraID});
    EntityManager::instance->addEntity(cameraEntity);
    cameraEntity->transform->translation = glm::vec3(0.0, 0.0, 10.0);
};