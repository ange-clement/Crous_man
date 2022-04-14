#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>
#include <common/meshGenerator.hpp>
#include <common/texture.hpp>

#include "../ECS/EntityManager.hpp"
#include "../ECS/Entity.hpp"
#include "../ECS/Bitmap.hpp"

#include "../Transform.hpp"

#include "../Components/Mesh.hpp"
#include "../Components/Renderer.hpp"
#include "../Components/PointLight.hpp"
#include "../Components/Camera.hpp"

#include "ECS_test.hpp"

void createSceneECS() {
    
    Entity* test2 = new Entity({SystemIDs::SpinID});
    EntityManager::instance->addEntity(test2);

    Entity* plane = new Entity({SystemIDs::MeshID, SystemIDs::RendererID});
    EntityManager::instance->addEntity(plane);
    plane->transform->translation = glm::vec3(0.0, -1.0, 0.0);
    plane->transform->scaling = glm::vec3(100.0, 100.0, 1.0);
    plane->transform->rotation.setRotation(-3.141592653*0.5, glm::vec3(1.0, 0.0, 0.0));
    unsigned short planeMeshID = EntityManager::instance->getComponentId(SystemIDs::MeshID, plane->id);
    Mesh* planeMesh = &EntityManager::instance->meshComponents[planeMeshID];
    quad(planeMesh->indexed_vertices, planeMesh->normals, planeMesh->UV, planeMesh->indices, planeMesh->triangles);
    unsigned short planeRendererID = EntityManager::instance->getComponentId(SystemIDs::RendererID, plane->id);
    RendererSystem* rendererSystem = dynamic_cast<RendererSystem*>(EntityManager::instance->systems[SystemIDs::RendererID]);
    rendererSystem->initBuffers(planeRendererID, plane->id);
    Renderer* planeRenderer = rendererSystem->getRenderer(planeRendererID);
    planeRenderer->diffuseBuffer  = loadTextureFromPPM("../ressources/earth.ppm");
    planeRenderer->specularBuffer = loadTextureFromPGM("../ressources/heightmap.pgm");

    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 10; j++) {
            for (int k = 0; k < 5; k++) {
                Entity* test = new Entity({SystemIDs::MeshID, SystemIDs::RendererID});
                EntityManager::instance->addEntity(test);
                test2->addChildren(test);
                test->transform->translation = glm::vec3(i*2.0, j*2.0, k*2.0);
                unsigned short testMeshID = EntityManager::instance->getComponentId(SystemIDs::MeshID, test->id);
                EntityManager::instance->meshComponents[testMeshID].loadFromFile("../ressources/suzanne.off", false);
                unsigned short testRendererID = EntityManager::instance->getComponentId(SystemIDs::RendererID, test->id);
                dynamic_cast<RendererSystem*>(EntityManager::instance->systems[SystemIDs::RendererID])->initBuffers(testRendererID, test->id);

                //std::cout << "\r" << k+j*10+i*100 << std::flush;
            }
        }
    }
    
    Entity* light = new Entity({SystemIDs::PointLightID});
    EntityManager::instance->addEntity(light);
    light->transform->translation = glm::vec3(5.0, 3.0, 0.0);
    unsigned short lightPointLightID = EntityManager::instance->getComponentId(SystemIDs::PointLightID, light->id);
    PointLight* lightPL = dynamic_cast<PointLightSystem*>(EntityManager::instance->systems[SystemIDs::PointLightID])->getPointLight(lightPointLightID);
    lightPL->color = glm::vec3(0.3, 0.7, 0.9);
    
    Entity* light2 = new Entity({SystemIDs::PointLightID});
    EntityManager::instance->addEntity(light2);
    light2->transform->translation = glm::vec3(-5.0, 3.0, 0.0);
    unsigned short light2PointLightID = EntityManager::instance->getComponentId(SystemIDs::PointLightID, light2->id);
    PointLight* light2PL = dynamic_cast<PointLightSystem*>(EntityManager::instance->systems[SystemIDs::PointLightID])->getPointLight(light2PointLightID);
    light2PL->color = glm::vec3(0.9, 0.7, 0.3);

    Entity* light3 = new Entity({SystemIDs::PointLightID});
    EntityManager::instance->addEntity(light3);
    light3->transform->translation = glm::vec3(0.0, 100.0, 0.0);
    unsigned short light3PointLightID = EntityManager::instance->getComponentId(SystemIDs::PointLightID, light3->id);
    PointLight* light3PL = dynamic_cast<PointLightSystem*>(EntityManager::instance->systems[SystemIDs::PointLightID])->getPointLight(light3PointLightID);
    light3PL->linear = 0.01;
    light3PL->quadratic = 0.0;

    Entity* cameraEntity = new Entity({SystemIDs::CameraID, SystemIDs::FlyingControllerID});
    EntityManager::instance->addEntity(cameraEntity);
    cameraEntity->transform->translation = glm::vec3(0.0, 0.0, -10.0);
    dynamic_cast<CameraSystem*>(EntityManager::instance->systems[SystemIDs::CameraID])->setScreenCamera(cameraEntity->id);
};