#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "../SceneObjects/Camera.hpp"
#include "../Scene.hpp"
#include "../ECS/EntityManager.hpp"
#include "../ECS/Entity.hpp"
#include "../ECS/Bitmap.hpp"

#include "../SceneObjects/UtilObjects/CharacterControlObject.hpp"
#include "../SceneObjects/UtilObjects/MouseVerticalControlObject.hpp"
#include "../SceneObjects/Terrain.hpp"
#include "../SceneObjects/LODObject.hpp"
#include "../Transform.hpp"
#include "../Mesh.hpp"

#include "Object_control.hpp"

#include "../SceneObjects/PointLight.hpp"
#include "../SceneObjects/Camera.hpp"

SceneObject* createSceneObject(Camera*& camera, std::vector<PointLight*>& pointLights) {
    SceneObject* scene = new SceneObject();

    SceneObject* terrain = new Terrain();
    terrain->transform->scaling *= 100.0;
    terrain->transform->translation = glm::vec3(-50.0, -10.0, -50.0);
    scene->addChildren(terrain);

    SceneObject* player = new CharacterControlObject(10.0, 0.12);
    player->loadMesh("../ressources/suzanne.off", false);
    player->loadShaders();
    scene->addChildren(player);

    SceneObject* cameraVerticalContainer = new MouseVerticalControlObject(0.12);
    cameraVerticalContainer->transform->setLocalPosition(glm::vec3(0.0, .5, 0.0));
    player->addChildren(cameraVerticalContainer);

    std::vector<std::string> files;
    std::vector<bool> hasNormal;
    files.push_back("../ressources/elephant_n.off");
    hasNormal.push_back(true);
    files.push_back("../ressources/camel_n.off");
    hasNormal.push_back(true);
    files.push_back("../ressources/avion_n.off");
    hasNormal.push_back(true);
    files.push_back("../ressources/suzanne.off");
    hasNormal.push_back(false);
    
    SceneObject* lod = new LODObject(files, hasNormal);
    scene->addChildren(lod);



    camera = new Camera();
    camera->transform->setLocalPosition(glm::vec3(0.0, 0.0, -15.0));
    cameraVerticalContainer->addChildren(camera);


    PointLight* light = new PointLight();
    light->transform->translation = glm::vec3(5.0, 5.0, 5.0);
    pointLights.push_back(light);
    scene->addChildren(light);





    Entity* test2 = new Entity();
    EntityManager::instance->addEntity(test2);

    Entity* test = new Entity({SystemIDs::SpinID});
    EntityManager::instance->addEntity(test);
    test2->addChildren(test);
    



    return scene;
};