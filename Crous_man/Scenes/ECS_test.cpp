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
#include "../ECS/EntityBuilder.hpp"
#include "../ECS/Bitmap.hpp"

#include "../Transform.hpp"

#include "../Components/Mesh.hpp"
#include "../Components/Renderer.hpp"
#include "../Components/PointLight.hpp"
#include "../Components/Camera.hpp"

#include "ECS_test.hpp"

void createSceneECS() {
    
    Entity* monkeContainer = (new EntityBuilder({ SystemIDs::SpinID }))->build();

    Entity* plane = (new EntityBuilder({ SystemIDs::MeshID, SystemIDs::RendererID }))
        ->setTranslation(glm::vec3(0.0, -1.0, 0.0))
        ->setScale(glm::vec3(100.0, 100.0, 1.0))
        ->setRotation(-3.141592653 * 0.5, glm::vec3(1.0, 0.0, 0.0))
        ->setMeshAsQuad()
        ->updateRenderer()
        ->setRendererDiffuseSpecular("../ressources/earth.ppm", "../ressources/heightmap.pgm")
        ->build();

    int nbI = 2;
    int nbJ = 2;
    int nbK = 2;
    for (int i = 0; i < nbI; i++) {
        for (int j = 0; j < nbJ; j++) {
            for (int k = 0; k < nbK; k++) {
                Entity* monke = (new EntityBuilder({ SystemIDs::MeshID, SystemIDs::RendererID }))
                    ->setChildOf(monkeContainer)
                    ->setTranslation(glm::vec3(i * 2.0, j * 2.0, k * 2.0))
                    ->setMeshAsFile("../ressources/suzanne.off", false)
                    ->updateRenderer()
                    ->build();

                std::cout << "\r" << k + j*nbK + i*nbJ*nbK << " / " << nbK*nbJ*nbI << std::flush;
            }
        }
    }
    std::cout << "\r                                                                             " << std::endl;

    Entity* explosionCube = (new EntityBuilder({ SystemIDs::MeshID, SystemIDs::RendererID, SystemIDs::DestructibleID }))
        ->setTranslation(glm::vec3(1.0, 1.0, 10.0))
        ->setMeshAsFilePLY("../ressources/fragment/cubeFragment.ply")
        ->setDestructibleMeshes({
            "../ressources/fragment/cubeFragment1.ply",
            "../ressources/fragment/cubeFragment2.ply",
            "../ressources/fragment/cubeFragment3.ply",
            "../ressources/fragment/cubeFragment4.ply",
            "../ressources/fragment/cubeFragment5.ply",
            "../ressources/fragment/cubeFragment6.ply",
            "../ressources/fragment/cubeFragment7.ply",
            "../ressources/fragment/cubeFragment8.ply",
            "../ressources/fragment/cubeFragment9.ply",
            "../ressources/fragment/cubeFragment10.ply",
            "../ressources/fragment/cubeFragment11.ply",
            "../ressources/fragment/cubeFragment12.ply",
            "../ressources/fragment/cubeFragment13.ply", })
        ->updateRenderer()
        ->setRendererDiffuseColor(glm::vec3(1.0, 1.0, 1.0))
        ->build();
    
    Entity* blueLight = (new EntityBuilder({ SystemIDs::PointLightID }))
        ->setTranslation(glm::vec3(5.0, 3.0, 0.0))
        ->setLightColor(glm::vec3(0.3, 0.7, 0.9))
        ->build();

    Entity* redLight2 = (new EntityBuilder({ SystemIDs::PointLightID }))
        ->setTranslation(glm::vec3(-5.0, 3.0, 0.0))
        ->setLightColor(glm::vec3(0.9, 0.7, 0.3))
        ->build();

    Entity* topLight2 = (new EntityBuilder({ SystemIDs::PointLightID }))
        ->setTranslation(glm::vec3(0.0, 100.0, 0.0))
        ->setLightLinear(0.01)
        ->setLightQuadratic(0.0)
        ->build();

    Entity* cameraEntity = (new EntityBuilder({ SystemIDs::CameraID, SystemIDs::FlyingControllerID }))
        ->setTranslation(glm::vec3(0.0, 0.0, -10.0))
        ->setAsScreenCamera()
        ->build();
};