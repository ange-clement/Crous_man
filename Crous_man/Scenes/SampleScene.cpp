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
#include "SampleScene.hpp"

#include "../SceneObjects/PointLight.hpp"
#include "../Transform.hpp"
#include "../Mesh.hpp"

SceneObject* createSceneMonkey(Camera** camera, std::vector<PointLight*>& pointLights) {
    SceneObject* scene = new SceneObject();

    SceneObject* empty = new SceneObject();
    scene->addChildren(empty);

    for (int x = -5; x <= 5; x+=2) {
        for (int y = -5; y <= 5; y+=2) {
            for (int z = -5; z <= 5; z+=2) {
                SceneObject* children = new SceneObject();
                empty->addChildren(children);

                children->transform->setLocalPosition(glm::vec3(x*2, y*2, z*2));

                children->loadMesh("../ressources/suzanne.off");
                children->loadShaders();
            }
        }
    }

    *camera = new Camera();
    (*camera)->transform->setLocalPosition(glm::vec3(0.0, 0.0, 0.0));
    scene->addChildren(*camera);

    PointLight* light = new PointLight();
    light->transform->translation = glm::vec3(5.0, 15.0, 5.0);
    pointLights.push_back(light);
    scene->addChildren(light);

    PointLight* light2 = new PointLight();
    light2->transform->translation = glm::vec3(-10.0, 0.0, 0.0);
    pointLights.push_back(light2);
    scene->addChildren(light2);

    return scene;
};