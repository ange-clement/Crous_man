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
#include "Systeme_solaire.hpp"

#include "../SceneObjects/PointLight.hpp"
#include "../Transform.hpp"
#include "../Mesh.hpp"

#include "../SceneObjects/Solaire/Sun.hpp"
#include "../SceneObjects/Solaire/Earth.hpp"
#include "../SceneObjects/Solaire/Moon.hpp"
#include "../SceneObjects/Solaire/SpinningObject.hpp"

SceneObject* createSceneSolaire(Camera*& camera, std::vector<PointLight*>& pointLights) {
    SceneObject* scene = new SceneObject();

    SceneObject* sunContainer = new SpinningObject(-0.01);
    scene->addChildren(sunContainer);

    SceneObject* soleil = new Sun();
    soleil->transform->scaling *= 10;
    sunContainer->addChildren(soleil);

    PointLight* light = new PointLight();
    pointLights.push_back(light);
    soleil->addChildren(light);



    SceneObject* earthMoonCenterContainer = new SpinningObject(0.2);
    scene->addChildren(earthMoonCenterContainer);

    SceneObject* earthCenterContainer = new SceneObject();
    earthCenterContainer->transform->translation = glm::vec3(15.0, 0.0, 0.0);
    earthMoonCenterContainer->addChildren(earthCenterContainer);

    SceneObject* earthContainer = new SpinningObject(1.0);
    earthCenterContainer->addChildren(earthContainer);

    SceneObject* earth = new Earth();
    earth->transform->rotation.setRotation(0.3, glm::vec3(1.0, 0.0, 0.0));
    earth->transform->scaling *= 1.0;
    earthContainer->addChildren(earth);
    
    

    SceneObject* moonCenterContainer = new SpinningObject(0.8);
    earthCenterContainer->addChildren(moonCenterContainer);

    SceneObject* moon = new Moon();
    moon->transform->translation = glm::vec3(2.0, 0.0, 0.0);
    moon->transform->rotation.setRotation(0.3, glm::vec3(1.0, 0.0, 1.0));
    moon->transform->rotation.combineRotation(3.14, glm::vec3(0.0, 1.0, 0.0));
    moon->transform->scaling *= 0.2;
    moonCenterContainer->addChildren(moon);



    camera = new Camera();
    camera->transform->translation = glm::vec3(0.0, 0.0, -30.0);
    scene->addChildren(camera);

    return scene;
};