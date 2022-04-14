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
#include "../ECS/Bitmap.hpp"
#include "../ECS/Entity.hpp"

#include "../Transform.hpp"

#include "PointLight.hpp"

PointLightSystem::PointLightSystem() : ComponentSystem(){
    requiredComponentsBitmap = new Bitmap({SystemIDs::PointLightID});
}

PointLightSystem::~PointLightSystem() {

}

void PointLightSystem::initialize(unsigned short i, unsigned short entityID) {
    
}

void PointLightSystem::update(unsigned short i, unsigned short entityID) {

}

void PointLightSystem::addEntityComponent() {
    EntityManager::instance->pointLightComponents.push_back(PointLight());
}

PointLight* PointLightSystem::getPointLight(unsigned short i) {
    return &EntityManager::instance->pointLightComponents[i];
}