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
#include "../SoundManager.hpp"

#include "../Components/Mesh.hpp"
#include "../Components/Renderer.hpp"
#include "../Components/PointLight.hpp"
#include "../Components/Camera.hpp"
#include "../Components/Collider.hpp"

#include "ECS_test.hpp"

void createSceneECS() {
    
    Entity* monkeContainer = (new EntityBuilder({ SystemIDs::SpinID }))->build();

    Entity* plane = (new EntityBuilder({ SystemIDs::ColliderID, SystemIDs::MeshID, SystemIDs::RendererID }))
        ->setTranslation(glm::vec3(0.0, -1.0, 0.0))
        ->setScale(glm::vec3(100.0, 100.0, 1.0))
        ->setRotation(-3.141592653 * 0.5, glm::vec3(1.0, 0.0, 0.0))
        ->setMeshAsQuad()
        ->updateRenderer()
        ->setRendererDiffuseSpecular("../ressources/Textures/earth.ppm", "../ressources/Textures/heightmap.pgm")
        ->setColliderType(colliderType::Sphere)
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
                    ->setMeshAsFile("../ressources/Models/suzanne.off", false)
                    ->updateRenderer()
                    ->build();

                std::cout << "\r" << k + j*nbK + i*nbJ*nbK << " / " << nbK*nbJ*nbI << std::flush;
            }
        }
    }
    std::cout << "\r                                                                             " << std::endl;

    Entity* explosionCube = (new EntityBuilder({ SystemIDs::ColliderID, SystemIDs::MeshID, SystemIDs::RendererID, SystemIDs::DestructibleID }))
        ->setTranslation(glm::vec3(1.0, 3.0, 10.0))
        ->setMeshAsFilePLY("../ressources/Models/fragment/cubeFragment.ply")
        ->addDestructibleMeshes({
            "../ressources/Models/fragment/cubeFragment1.ply",
            "../ressources/Models/fragment/cubeFragment2.ply",
            "../ressources/Models/fragment/cubeFragment3.ply",
            "../ressources/Models/fragment/cubeFragment4.ply",
            "../ressources/Models/fragment/cubeFragment5.ply",
            "../ressources/Models/fragment/cubeFragment6.ply",
            "../ressources/Models/fragment/cubeFragment7.ply",
            "../ressources/Models/fragment/cubeFragment8.ply",
            "../ressources/Models/fragment/cubeFragment9.ply",
            "../ressources/Models/fragment/cubeFragment10.ply",
            "../ressources/Models/fragment/cubeFragment11.ply",
            "../ressources/Models/fragment/cubeFragment12.ply" })
        ->addDestructibleMeshes({
            "../ressources/Models/fragment/cubeFragment13.ply"
            }, true)
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

    SoundManager::instance->play("../ressources/Sounds/start.wav");

};

void createSceneCollider() {
   
    Entity* monke = (new EntityBuilder({ SystemIDs::ColliderID, SystemIDs::RigidBodyID, SystemIDs::MeshID, SystemIDs::RendererID, SystemIDs::SimplePlayerControllerID }))
        ->setTranslation(glm::vec3(2.0, 2.0, 2.0))
        ->setMeshAsFile("../ressources/Models/suzanne.off", false)
        ->setRigidBodyMass(2.0f)
        ->updateRenderer()
        ->fitOBBColliderToMesh()
        ->setRenderingCollider()
        ->build();

    Entity* monke2 = (new EntityBuilder({SystemIDs::RigidBodyID, SystemIDs::ColliderID, SystemIDs::MeshID, SystemIDs::RendererID, SystemIDs::SimplePlayerControllerID}))
        ->setTranslation(glm::vec3(10.0, 2.0, 2.0))
        ->setMeshAsFile("../ressources/Models/suzanne.off", false)
        ->updateRenderer()
        ->fitSphereColliderToMesh()
        ->setRenderingCollider()
        ->build();

    Entity* monke3 = (new EntityBuilder({ SystemIDs::ColliderID, SystemIDs::MeshID, SystemIDs::RendererID}))
        ->setTranslation(glm::vec3(10.0, 2.0, 10.0))
        ->setMeshAsFile("../ressources/Models/suzanne.off", false)
        ->setRotation(-3.141592653 * 0.3, glm::vec3(0.0, 1.0, 0.0))
        ->updateRenderer()
        ->fitAABBColliderToMesh()
        ->setRenderingCollider()
        ->build();
    
    Entity* monke4 = (new EntityBuilder({SystemIDs::ColliderID, SystemIDs::MeshID, SystemIDs::RendererID}))
        ->setTranslation(glm::vec3(10.0, 20.0, 2.0))
        ->setScale(glm::vec3(10.0, 10.0, 10.0))
        ->setRotation(-3.141592653* .1f, glm::vec3(1.0, 0.0, 0.0))
        ->setMeshAsFile("../ressources/Models/suzanne.off", false)
        ->updateRenderer()
        ->fitOBBColliderToMesh()
        ->setRenderingCollider()
        ->build();

    Entity* plane = (new EntityBuilder({ SystemIDs::RigidBodyID, SystemIDs::ColliderID, SystemIDs::MeshID, SystemIDs::RendererID }))
        ->setTranslation(glm::vec3(0.0, -1.0, 0.0))
        ->setScale(glm::vec3(100.0, 100.0, 1.0))
        ->setRotation(-3.141592653 * 0.5, glm::vec3(1.0, 0.0, 0.0))
        ->setRigidBodyStatic(true)
        ->setMeshAsQuad()
        //->setMeshAsFilePLY("../ressources/Models/fragment/cubeFragment.ply")
        ->updateRenderer()
        ->setRendererDiffuseSpecular("../ressources/Textures/earth.ppm", "../ressources/Textures/heightmap.pgm")
        ->fitAABBColliderToMesh()
        ->setRenderingCollider()
        ->build();
   

    Entity* topLight = (new EntityBuilder({ SystemIDs::PointLightID }))
        ->setTranslation(glm::vec3(0.0, 100.0, 0.0))
        ->setLightLinear(0.01)
        ->setLightQuadratic(0.0)
        ->build();

    Entity* cameraEntity = (new EntityBuilder({ SystemIDs::CameraID, SystemIDs::FlyingControllerID }))
        ->setTranslation(glm::vec3(0.0, 0.0, -10.0))
        ->setAsScreenCamera()
        ->setAsAudioListener()
        ->build();
}


void createSceneGame() {

    // CrousMan
    Entity* rotatingCenterForCamera = (new EntityBuilder({}))
        ->setTranslation(glm::vec3(0.0f, 15.0f, 0.0f))
        ->build();

    Entity* wantedCameraPosition = (new EntityBuilder({}))
        ->setTranslation(glm::vec3(0.0f, 0.0f, -50.0f))
        //->setRotation(1.57079632679, glm::vec3(0.0f, 1.0f, 0.0f))
        ->setChildOf(rotatingCenterForCamera)
        ->build();

    // Camera

    Entity* cameraEntity = (new EntityBuilder({ SystemIDs::CameraID}))
        ->setAsScreenCamera()
        ->setAsAudioListener()
        ->build();
    
    Entity* laser = (new EntityBuilder({ SystemIDs::MeshID, SystemIDs::RendererID }))
        ->setMeshAsCube()
        ->updateRenderer()
        ->setRendererDiffuse("../ressources/Textures/laser.ppm")
        ->setRendererSpecularValue(0.0f)
        ->setRendererCastShadows(false)
        ->build();
    
    Entity* laserLight = (new EntityBuilder({ SystemIDs::PointLightID }))
        ->setChildOf(laser)
        ->setLightColor(glm::vec3(.958, .198, .375))
        ->setLightLinear(0.05)
        ->setLightQuadratic(0.00)
        ->build();

    Entity* saucisse = (new EntityBuilder({ SystemIDs::MeshID, SystemIDs::RendererID }))
        ->setTranslation(glm::vec3(0.0f, 7.0f, 0.0f))
        ->setMeshAsFilePLY("../ressources/Models/saucisseCentre.ply")
        ->updateRenderer()
        ->setRendererDiffuse("../ressources/Textures/saucisseColor.ppm")
        ->build();

    Entity* doughnutSaucisse = (new EntityBuilder({ SystemIDs::MeshID, SystemIDs::RendererID, SystemIDs::ColliderID, SystemIDs::RigidBodyID, SystemIDs::CrousManControllerID }))
        ->setTranslation(glm::vec3(0.0f, 6.0f, 0.0f))
        ->setCrousManControllerRotatingCenterForCamera(rotatingCenterForCamera)
        ->setCrousManControllerCameraTarget(wantedCameraPosition)
        ->setCrousManControllerCameraEntity(cameraEntity)
        ->setCrousManControllerSaucisseEntity(saucisse)
        ->setCrousManControllerLaserEntity(laser)
        ->setMeshAsFilePLY("../ressources/Models/dougnut.ply")
        ->updateRenderer()
        ->setRendererDiffuse("../ressources/Textures/dougnutColor.ppm")
        ->setRigidBodyMass(1.0f)
        ->fitOBBColliderToMesh()
        ->setRenderingCollider()
        ->build();

    Entity* crousLight = (new EntityBuilder({ SystemIDs::PointLightID }))
        ->setChildOf(saucisse)
        ->setTranslation(glm::vec3(0.0, -4.0, 0.0))
        ->setLightColor(glm::vec3(.958, .985, .938))
        ->setLightLinear(0.2)
        ->setLightQuadratic(0.04)
        ->build();


    


    // Scene

    Entity* plane = (new EntityBuilder({ SystemIDs::RigidBodyID, SystemIDs::ColliderID, SystemIDs::MeshID, SystemIDs::RendererID }))
        ->setTranslation(glm::vec3(0.0, -1.0, 0.0))
        ->setScale(glm::vec3(200.0, 200.0, 1.0))
        ->setRotation(-3.141592653 * 0.5, glm::vec3(1.0, 0.0, 0.0))
        ->setRigidBodyStatic(true)
        //->setMeshAsQuad()
        ->setMeshAsCube()
        ->updateRenderer()
        ->setRendererDiffuseSpecular("../ressources/Textures/earth.ppm", "../ressources/Textures/heightmap.pgm")
        ->fitAABBColliderToMesh()
        ->setRenderingCollider()
        ->build();
    
    Entity* explosionCube = (new EntityBuilder({ SystemIDs::ColliderID, SystemIDs::MeshID, SystemIDs::RendererID, SystemIDs::DestructibleID }))
        ->setTranslation(glm::vec3(0.0f, 10.0f, 50.0f))
        ->setScale(glm::vec3(10.0f, 10.0f, 10.0f))
        ->setMeshAsFilePLY("../ressources/Models/fragment/cubeFragment.ply")
        ->fitAABBColliderToMesh()
        ->setRenderingCollider()
        ->addDestructibleMeshes({
            "../ressources/Models/fragment/cubeFragment1.ply",
            "../ressources/Models/fragment/cubeFragment2.ply",
            "../ressources/Models/fragment/cubeFragment3.ply",
            "../ressources/Models/fragment/cubeFragment4.ply",
            "../ressources/Models/fragment/cubeFragment5.ply",
            "../ressources/Models/fragment/cubeFragment6.ply",
            "../ressources/Models/fragment/cubeFragment7.ply",
            "../ressources/Models/fragment/cubeFragment8.ply",
            "../ressources/Models/fragment/cubeFragment9.ply",
            "../ressources/Models/fragment/cubeFragment10.ply",
            "../ressources/Models/fragment/cubeFragment11.ply",
            "../ressources/Models/fragment/cubeFragment12.ply" })
            ->addDestructibleMeshes({
                "../ressources/Models/fragment/cubeFragment13.ply"
                }, true)
        ->updateRenderer()
        ->setRendererDiffuseColor(glm::vec3(.958, .985, .938))
        ->build();

    


    Entity* topLight = (new EntityBuilder({ SystemIDs::PointLightID }))
        ->setTranslation(glm::vec3(0.0, 100.0, 0.0))
        ->setLightLinear(0.01)
        ->setLightQuadratic(0.004)
        ->build();
}