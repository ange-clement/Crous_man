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
#include "../SoundManager.hpp"

#include "../Components/Mesh.hpp"
#include "../Components/Renderer.hpp"
#include "../Components/PointLight.hpp"
#include "../Components/Camera.hpp"
#include "../Components/Collider.hpp"

#include "ECS_test.hpp"
#include "../ECS/EntityBuilder.hpp"


#define ADD_DEFAULT_DESTRUCTIVE_MESHES ->addDestructibleMeshes({       \
            "../ressources/Models/fragment/cubeFragment1.ply",      \
            "../ressources/Models/fragment/cubeFragment2.ply",      \
            "../ressources/Models/fragment/cubeFragment3.ply",      \
            "../ressources/Models/fragment/cubeFragment4.ply",      \
            "../ressources/Models/fragment/cubeFragment5.ply",      \
            "../ressources/Models/fragment/cubeFragment6.ply",      \
            "../ressources/Models/fragment/cubeFragment7.ply",      \
            "../ressources/Models/fragment/cubeFragment8.ply",      \
            "../ressources/Models/fragment/cubeFragment9.ply",      \
            "../ressources/Models/fragment/cubeFragment10.ply",     \
            "../ressources/Models/fragment/cubeFragment11.ply",     \
            "../ressources/Models/fragment/cubeFragment12.ply" })   \
            ->addDestructibleMeshes({                               \
                "../ressources/Models/fragment/cubeFragment13.ply"  \
                }, true)

#define ADD_LESS_DESTRUCTIVE_MESHES ->addDestructibleMeshes({       \
            "../ressources/Models/fragment/cubeFragment2.ply",      \
            "../ressources/Models/fragment/cubeFragment3.ply",      \
            "../ressources/Models/fragment/cubeFragment7.ply",      \
            "../ressources/Models/fragment/cubeFragment12.ply" })

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
        ADD_DEFAULT_DESTRUCTIVE_MESHES
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

    //7
    Entity* doughnutSaucisse = (new EntityBuilder({ SystemIDs::MeshID, SystemIDs::RendererID, SystemIDs::ColliderID, SystemIDs::RigidBodyID, SystemIDs::CrousManControllerID }))
        ->setTranslation(glm::vec3(0.0f, 20.0f, 0.0f))
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

    Entity* croustopLight = (new EntityBuilder({ SystemIDs::PointLightID }))
        ->setChildOf(saucisse)
        ->setTranslation(glm::vec3(0.0, 100.0, 0.0))
        ->setLightLinear(0.01)
        ->setLightQuadratic(0.0001)
        ->setLightColor(glm::vec3(.984, .948, .628))
        ->build();
   
    //10
    Entity* monke = (new EntityBuilder({ SystemIDs::ColliderID, SystemIDs::RigidBodyID, SystemIDs::MeshID, SystemIDs::RendererID, SystemIDs::SimplePlayerControllerID }))
        ->setTranslation(glm::vec3(2.0, 2.0, 2.0))
        ->setMeshAsFile("../ressources/Models/suzanne.off", false)
        ->setRigidBodyMass(2.0f)
        ->updateRenderer()
        ->fitOBBColliderToMesh()
        ->setRenderingCollider()
        ->build();

    //11
    Entity* monke2 = (new EntityBuilder({SystemIDs::RigidBodyID, SystemIDs::ColliderID, SystemIDs::MeshID, SystemIDs::RendererID, SystemIDs::SimplePlayerControllerID}))
        ->setTranslation(glm::vec3(10.0, 2.0, 2.0))
        ->setMeshAsFile("../ressources/Models/suzanne.off", false)
        ->updateRenderer()
        ->fitSphereColliderToMesh()
        ->setRenderingCollider()
        ->build();

    //12
    Entity* monke3 = (new EntityBuilder({ SystemIDs::ColliderID, SystemIDs::MeshID, SystemIDs::RendererID}))
        ->setTranslation(glm::vec3(10.0, 2.0, 10.0))
        ->setMeshAsFile("../ressources/Models/suzanne.off", false)
        ->setRotation(-3.141592653 * 0.3, glm::vec3(0.0, 1.0, 0.0))
        ->updateRenderer()
        ->fitAABBColliderToMesh()
        ->setRenderingCollider()
        ->build();
    
    //13
    Entity* monke4 = (new EntityBuilder({SystemIDs::ColliderID, SystemIDs::MeshID, SystemIDs::RendererID}))
        ->setTranslation(glm::vec3(10.0, 20.0, 2.0))
        ->setScale(glm::vec3(10.0, 10.0, 10.0))
        ->setRotation(-3.141592653* .1f, glm::vec3(1.0, 0.0, 0.0))
        ->setMeshAsFile("../ressources/Models/suzanne.off", false)
        ->updateRenderer()
        ->fitOBBColliderToMesh()
        ->setRenderingCollider()
        ->build();

    //14
    Entity* plane = (new EntityBuilder({ SystemIDs::RigidBodyID, SystemIDs::ColliderID, SystemIDs::MeshID, SystemIDs::RendererID }))
        ->setTranslation(glm::vec3(0.0, -1.0, 0.0))
        ->setScale(glm::vec3(100.0, 100.0, 1.0))
        ->setRotation(-3.141592653 * 0.5, glm::vec3(1.0, 0.0, 0.0))
        ->setRigidBodyStatic(true)
        ->setMeshAsQuad()
        //->setMeshAsFilePLY("../ressources/Models/fragment/cubeFragment.ply")
        ->updateRenderer()
        ->setRendererDiffuseSpecular("../ressources/Textures/earth.ppm", "../ressources/Textures/heightmap.pgm")
        ->fitOBBColliderToMesh()
        ->setRenderingCollider()
        ->build();
   

    Entity* topLight = (new EntityBuilder({ SystemIDs::PointLightID }))
        ->setTranslation(glm::vec3(0.0, 100.0, 0.0))
        ->setLightLinear(0.01)
        ->setLightQuadratic(0.0)
        ->build();

    // Entity* cameraEntity = (new EntityBuilder({ SystemIDs::CameraID, SystemIDs::FlyingControllerID }))
    //     ->setTranslation(glm::vec3(0.0, 0.0, -10.0))
    //     ->setAsScreenCamera()
    //     ->setAsAudioListener()
    //     ->build();
}

void addTree(glm::vec3 pos, glm::vec3 scale = glm::vec3(1.0f, 1.0f, 1.0f)) {
    float hedgeSpec = 0.0f;
    glm::vec3 treeLeafColor = glm::vec3(0.239, 0.981, 0.183);
    glm::vec3 treeColor = glm::vec3(0.631, 0.220, 0.192);

    Entity* tree1 = (new EntityBuilder({ SystemIDs::MeshID, SystemIDs::RendererID }))
        ->setTranslation(pos)
        ->setRotation(rand(), glm::vec3(0.0f, 1.0f, 0.0f))
        ->setScale(scale)
        ->setMeshAsFilePLY("../ressources/Models/Scene/tree.ply")
        ->updateRenderer()
        ->setRendererDiffuseColor(treeColor)
        ->setRendererSpecularValue(hedgeSpec)
        ->build();
    Entity* treeLeaf1 = (new EntityBuilder({ SystemIDs::MeshID, SystemIDs::RendererID, SystemIDs::ColliderID, SystemIDs::DestructibleID }))
        ->setChildOf(tree1)
        ->setTranslation(glm::vec3(0.0f, 10.0f, 0.0f))
        ->setMeshAsFilePLY("../ressources/Models/Scene/treeLeaf.ply")
        ->updateRenderer()
        ->setRendererDiffuseColor(treeLeafColor)
        ->setRendererSpecularValue(hedgeSpec)
        ->fitAABBColliderToMesh()
        ->setRenderingCollider()
        ADD_LESS_DESTRUCTIVE_MESHES
        ->setDestructibleFragmentScaling(glm::vec3(1.0f, 1.0f, 1.0f))
        ->setDestructibleFragmentColor(treeLeafColor)
        ->setDestructibleHealth(0.01f)
        ->build();
}

void addForest(int nbX = 5, int nbY = 5, float centerX = 50.0f, float centerY = -50.0f, float spacingX = 8.0f, float spacingY = 8.0f, float randomAmount = 5.0f, float scaleRandom = 0.2f) {
    float startX = centerX - nbX * spacingX * 0.5;
    float startY = centerY - nbY * spacingY * 0.5;
    for (int i = 0; i < nbX; i++) {
        for (int j = 0; j < nbY; j++) {
            addTree(glm::vec3(startX + spacingX * i, 0.4f, startY + spacingY * j) + glm::vec3(rand(), 0.0f, rand()) /(float) RAND_MAX * randomAmount, glm::vec3(1.0f + rand()/(float) RAND_MAX * scaleRandom));
        }
    }
}


void createSceneGame() {
    srand(time(NULL));

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

    //7
    Entity* doughnutSaucisse = (new EntityBuilder({ SystemIDs::MeshID, SystemIDs::RendererID, SystemIDs::ColliderID, SystemIDs::RigidBodyID, SystemIDs::CrousManControllerID }))
        ->setTranslation(glm::vec3(0.0f, 20.0f, 0.0f))
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

    Entity* croustopLight = (new EntityBuilder({ SystemIDs::PointLightID }))
        ->setChildOf(saucisse)
        ->setTranslation(glm::vec3(0.0, 100.0, 0.0))
        ->setLightLinear(0.01)
        ->setLightQuadratic(0.0001)
        ->setLightColor(glm::vec3(.984, .948, .628))
        ->build();


    


    // Scene

    glm::vec3 trotColor = glm::vec3(0.8f, 0.75f, 0.7f);
    float trotSpec = 0.4f;
    glm::vec3 routeColor = glm::vec3(0.3f, 0.3f, 0.3f);
    float routeSpec = 0.8f;

    float hedgeSpec = 0.0f;

    glm::vec3 lightColor = glm::vec3(.984, .948, .628);

    //10
    Entity* ground = (new EntityBuilder({ SystemIDs::RigidBodyID, SystemIDs::ColliderID, SystemIDs::MeshID, SystemIDs::RendererID }))
        ->setTranslation(glm::vec3(0.0, -5.0, 0.0))
        ->setScale(glm::vec3(500.0, 10.0, 500.0))
        ->setRigidBodyStatic(true)
        ->setMeshAsCube()
        ->updateRenderer()
        ->setRendererDiffuseColor(routeColor)
        ->setRendererSpecularValue(routeSpec)
        ->fitAABBColliderToMesh()
        ->setRenderingCollider()
        ->build();


    // Giratoire

    //11
    Entity* giratoire = (new EntityBuilder({ SystemIDs::RigidBodyID, SystemIDs::ColliderID, SystemIDs::MeshID, SystemIDs::RendererID }))
        ->setRigidBodyStatic(true)
        ->setMeshAsFilePLY("../ressources/Models/Scene/giratoire.ply")
        ->updateRenderer()
        ->setRendererDiffuse("../ressources/Textures/Scene/giratoireColor.ppm")
        ->setRendererSpecularValue(hedgeSpec)
        ->setColliderType(colliderType::Sphere)
        ->setColliderCenter(glm::vec3(0.0f, -43.0f, 0.0f))
        ->setColliderRadius(50.0f)
        ->setRenderingCollider()
        ->build();

    Entity* giratoireTrot = (new EntityBuilder({ SystemIDs::MeshID, SystemIDs::RendererID }))
        ->setChildOf(giratoire)
        ->setTranslation(glm::vec3(0.0f, 0.2f, 0.0f))
        ->setMeshAsFilePLY("../ressources/Models/Scene/giratoireTrot.ply")
        ->updateRenderer()
        ->setRendererDiffuse("../ressources/Textures/Scene/giratoireTrotColor.ppm")
        ->setRendererSpecularValue(trotSpec)
        ->build();

    Entity* trotGira1 = (new EntityBuilder({ SystemIDs::MeshID, SystemIDs::RendererID }))
        ->setTranslation(glm::vec3(0.0f, 0.4f, 0.0f))
        ->setMeshAsFilePLY("../ressources/Models/Scene/trotGira1.ply")
        ->updateRenderer()
        ->setRendererDiffuseColor(trotColor)
        ->setRendererSpecularValue(trotSpec)
        ->build();
    Entity* trotGira2 = (new EntityBuilder({ SystemIDs::MeshID, SystemIDs::RendererID }))
        ->setTranslation(glm::vec3(0.0f, 0.4f, 0.0f))
        ->setMeshAsFilePLY("../ressources/Models/Scene/trotGira2.ply")
        ->updateRenderer()
        ->setRendererDiffuseColor(trotColor)
        ->setRendererSpecularValue(trotSpec)
        ->build();
    Entity* trotGira3 = (new EntityBuilder({ SystemIDs::MeshID, SystemIDs::RendererID }))
        ->setTranslation(glm::vec3(0.0f, 0.4f, 0.0f))
        ->setMeshAsFilePLY("../ressources/Models/Scene/trotGira3.ply")
        ->updateRenderer()
        ->setRendererDiffuseColor(trotColor)
        ->setRendererSpecularValue(trotSpec)
        ->build();
    Entity* trotGira4 = (new EntityBuilder({ SystemIDs::MeshID, SystemIDs::RendererID }))
        ->setTranslation(glm::vec3(0.0f, 0.4f, 0.0f))
        ->setMeshAsFilePLY("../ressources/Models/Scene/trotGira4.ply")
        ->updateRenderer()
        ->setRendererDiffuseColor(trotColor)
        ->setRendererSpecularValue(trotSpec)
        ->build();
    //17
    Entity* trotGira5 = (new EntityBuilder({ SystemIDs::MeshID, SystemIDs::RendererID }))
        ->setTranslation(glm::vec3(0.0f, 0.4f, 0.0f))
        ->setMeshAsFilePLY("../ressources/Models/Scene/trotGira5.ply")
        ->updateRenderer()
        ->setRendererDiffuseColor(trotColor)
        ->setRendererSpecularValue(trotSpec)
        ->build();
    
    //void addForest(int nbX = 5, int nbY = 5, float centerX = 50.0f, float centerY = -50.0f, float spacingX = 8.0f, float spacingY = 8.0f, float randomAmount = 5.0f, float scaleRandom = 0.2f)
    addForest(2, 3, 50.0f, -50.0f, 10.0f, 15.0f, 8.0f);
    //23
    addForest(2, 1, -50.0f, -50.0f, 10.0f, 10.0f, 5.0f);
    //25
    addForest(1, 4, 25.0f, 100.0f, 10.0f, 10.0f, 15.0f);
    //29
    //30
    addTree(glm::vec3(7.0f, 0.4f, -60.0f));
    addTree(glm::vec3(7.0f, 0.4f, -70.0f));
    
    

    // Resto

    Entity* restoElevation = (new EntityBuilder({ SystemIDs::RigidBodyID, SystemIDs::ColliderID, SystemIDs::MeshID, SystemIDs::RendererID }))
        ->setRigidBodyStatic(true)
        ->setTranslation(glm::vec3(0.0f, 1.7f, 0.0f))
        ->setMeshAsFilePLY("../ressources/Models/Scene/restoElevation.ply")
        ->updateRenderer()
        ->setRendererDiffuseColor(glm::vec3(.850f, .619f, .698f))
        ->setRendererSpecularValue(trotSpec)
        ->setColliderType(colliderType::AABB)
        ->setColliderCenter(glm::vec3(185.0f, 0.2f, 55.0f))
        ->setColliderSize(glm::vec3(62.0f, 3.0f, 111.0f))
        ->setRenderingCollider()
        ->build();

    Entity* restoTrot = (new EntityBuilder({ SystemIDs::MeshID, SystemIDs::RendererID }))
        ->setTranslation(glm::vec3(0.0f, 0.4f, 0.0f))
        ->setMeshAsFilePLY("../ressources/Models/Scene/restoTrot.ply")
        ->updateRenderer()
        ->setRendererDiffuse("../ressources/Textures/Scene/restoTrotColor.ppm")
        ->setRendererSpecularValue(trotSpec)
        ->build();

    
    Entity* restoHedge = (new EntityBuilder({ SystemIDs::MeshID, SystemIDs::RendererID }))
        ->setTranslation(glm::vec3(0.0f, 3.2f, 0.0f))
        ->setMeshAsFilePLY("../ressources/Models/Scene/restoHedge.ply")
        ->updateRenderer()
        ->setRendererDiffuse("../ressources/Textures/Scene/restoHedgeColor.ppm")
        ->setRendererSpecularValue(hedgeSpec)
        ->build();
    Entity* restoHedge2 = (new EntityBuilder({ SystemIDs::MeshID, SystemIDs::RendererID }))
        ->setTranslation(glm::vec3(0.0f, 3.2f, 0.0f))
        ->setMeshAsFilePLY("../ressources/Models/Scene/restoHedge2.ply")
        ->updateRenderer()
        ->setRendererDiffuse("../ressources/Textures/Scene/restoHedge2Color.ppm")
        ->setRendererSpecularValue(hedgeSpec)
        ->build();


    Entity* resto =  (new EntityBuilder({ SystemIDs::DestructibleID, SystemIDs::RigidBodyID, SystemIDs::ColliderID }))
        ->setRigidBodyStatic(true)
        ->setTranslation(glm::vec3(180.0f, 25.0f, 75.0f))
        ->setColliderType(colliderType::AABB)
        ->setColliderCenter(glm::vec3(0.0f, 0.0f, 0.0f))
        ->setColliderSize(glm::vec3(75.0f, 20.0f, 75.0f))
        ->setRenderingCollider()
        ->build();

    Entity* restoFirst = (new EntityBuilder({ SystemIDs::RigidBodyID, SystemIDs::ColliderID, SystemIDs::MeshID, SystemIDs::RendererID, SystemIDs::DestructibleID }))
        ->setRigidBodyStatic(true)
        ->setRigidBodyMass(50.0f)
        ->setChildOf(resto)
        ->setTranslation(glm::vec3(0.0f, -15.0f, 0.0f))
        ->setMeshAsFilePLYCenter("../ressources/Models/Scene/restoFirst.ply")
        ->updateRenderer()
        ->setRendererDiffuse("../ressources/Textures/Scene/restoFirstColor.ppm")
        ->setRendererSpecularValue(hedgeSpec)

        ->setColliderType(colliderType::AABB)
        ->setColliderCenter(glm::vec3(0.0f, 0.0f, 0.0f))
        ->setColliderSize(glm::vec3(0.01f, 0.01f, 0.01f))
        ->setRenderingCollider()
        ADD_DEFAULT_DESTRUCTIVE_MESHES
        ->setDestructibleFragmentScaling(glm::vec3(30.0f, 10.0f, 30.0f))

        ->build();
    Entity* restoSecond = (new EntityBuilder({ SystemIDs::RigidBodyID, SystemIDs::ColliderID, SystemIDs::MeshID, SystemIDs::RendererID, SystemIDs::DestructibleID }))
        ->setRigidBodyStatic(true)
        ->setRigidBodyMass(100.0f)
        ->setChildOf(resto)
        ->setTranslation(glm::vec3(0.0f, 0.0f, 0.0f))
        ->setMeshAsFilePLYCenter("../ressources/Models/Scene/restoSecond.ply")
        ->updateRenderer()
        ->setRendererDiffuse("../ressources/Textures/Scene/restoSecondColor.ppm")
        ->setRendererSpecular("../ressources/Textures/Scene/restoSecondSpecular.pgm")

        ->setColliderType(colliderType::AABB)
        ->setColliderCenter(glm::vec3(0.0f, 0.0f, 0.0f))
        ->setColliderSize(glm::vec3(0.01f, 0.01f, 0.01f))
        ->setRenderingCollider()
        ADD_DEFAULT_DESTRUCTIVE_MESHES
        ->setDestructibleFragmentScaling(glm::vec3(30.0f, 10.0f, 30.0f))
        
        ->build();
    Entity* restoToit = (new EntityBuilder({ SystemIDs::RigidBodyID, SystemIDs::ColliderID, SystemIDs::MeshID, SystemIDs::RendererID, SystemIDs::DestructibleID }))
        ->setRigidBodyStatic(true)
        ->setRigidBodyMass(25.0f)
        ->setChildOf(resto)
        ->setTranslation(glm::vec3(0.0f, 12.5f, 0.0f))
        ->setMeshAsFilePLYCenter("../ressources/Models/Scene/restoToit.ply")
        ->updateRenderer()
        ->setRendererDiffuseColor(glm::vec3(.758f, .850f, .819f))
        ->setRendererSpecularValue(trotSpec)

        ->setColliderType(colliderType::AABB)
        ->setColliderCenter(glm::vec3(0.0f, 0.0f, 0.0f))
        ->setColliderSize(glm::vec3(0.01f, 0.01f, 0.01f))
        ->setRenderingCollider()
        ADD_DEFAULT_DESTRUCTIVE_MESHES
        ->setDestructibleFragmentScaling(glm::vec3(30.0f, 2.0f, 30.0f))
        
        ->build();

    


    Entity* restoPizza =  (new EntityBuilder({ SystemIDs::DestructibleID, SystemIDs::RigidBodyID, SystemIDs::ColliderID }))
        ->setRigidBodyStatic(true)
        ->setTranslation(glm::vec3(222.0f, 11.0f, -43.0f))
        ->setColliderType(colliderType::AABB)
        ->setColliderCenter(glm::vec3(0.0f, 0.0f, 0.0f))
        ->setColliderSize(glm::vec3(16.0f, 6.0f, 16.0f))
        ->setRenderingCollider()
        ->build();
    //40
    Entity* restoPizzaPremier = (new EntityBuilder({ SystemIDs::RigidBodyID, SystemIDs::ColliderID, SystemIDs::MeshID, SystemIDs::RendererID, SystemIDs::DestructibleID }))
        ->setRigidBodyStatic(true)
        ->setRigidBodyMass(20.0f)
        ->setChildOf(restoPizza)
        ->setTranslation(glm::vec3(0.0f, -3.0f, 0.0f))
        ->setMeshAsFilePLYCenter("../ressources/Models/Scene/restoPizzaFirst.ply")
        ->updateRenderer()
        ->setRendererDiffuseColor(trotColor)
        ->setRendererSpecularValue(trotSpec)

        ->setColliderType(colliderType::AABB)
        ->setColliderCenter(glm::vec3(0.0f, 0.0f, 0.0f))
        ->setColliderSize(glm::vec3(0.01f, 0.01f, 0.01f))
        ->setRenderingCollider()
        ADD_DEFAULT_DESTRUCTIVE_MESHES
        ->setDestructibleFragmentScaling(glm::vec3(15.0f, 3.0f, 15.0f))
        
        ->build();
    Entity* restoPizzaToit = (new EntityBuilder({ SystemIDs::RigidBodyID, SystemIDs::ColliderID, SystemIDs::MeshID, SystemIDs::RendererID, SystemIDs::DestructibleID }))
        ->setRigidBodyStatic(true)
        ->setRigidBodyMass(5.0f)
        ->setChildOf(restoPizza)
        ->setTranslation(glm::vec3(0.0f, 2.0f, 0.0f))
        ->setMeshAsFilePLYCenter("../ressources/Models/Scene/restoPizzaToit.ply")
        ->updateRenderer()
        ->setRendererDiffuseColor(glm::vec3(1.0f, 1.0f, 1.0f))
        ->setRendererSpecularValue(0.9f)

        ->setColliderType(colliderType::AABB)
        ->setColliderCenter(glm::vec3(0.0f, 0.0f, 0.0f))
        ->setColliderSize(glm::vec3(0.01f, 0.01f, 0.01f))
        ->setRenderingCollider()
        ADD_DEFAULT_DESTRUCTIVE_MESHES
        ->setDestructibleFragmentScaling(glm::vec3(10.0f, 2.0f, 10.0f))
        
        ->build();

    //42
    Entity* restoTopLight = (new EntityBuilder({ SystemIDs::PointLightID }))
        ->setChildOf(restoElevation)
        ->setTranslation(glm::vec3(0.0, 100.0, 0.0))
        ->setLightLinear(0.01)
        ->setLightQuadratic(0.004)
        ->setLightColor(lightColor)
        ->build();

    //void addForest(int nbX = 5, int nbY = 5, float centerX = 50.0f, float centerY = -50.0f, float spacingX = 8.0f, float spacingY = 8.0f, float randomAmount = 5.0f, float scaleRandom = 0.2f)
    addForest(2, 3, 150.0f, -80.0f, 20.0f, 10.0f, 8.0f);
    
    

    //48
    Entity* topLight = (new EntityBuilder({ SystemIDs::PointLightID }))
        ->setTranslation(glm::vec3(0.0, 100.0, 0.0))
        ->setLightLinear(0.01)
        ->setLightQuadratic(0.004)
        ->setLightColor(glm::vec3(.628, .948, .628))
        ->build();



    SoundManager::instance->playLoop("../ressources/Sounds/apokalipss.wav");
}