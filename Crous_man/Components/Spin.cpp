#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "../ECS/EntityManager.hpp"
#include "../ECS/Bitmap.hpp"
#include "../ECS/Entity.hpp"

#include "../Transform.hpp"

#include "Spin.hpp"

SpinSystem::SpinSystem() : ComponentSystem(){
    requiredComponentsBitmap = new Bitmap({SystemIDs::SpinID});
}

SpinSystem::~SpinSystem() {

}

void SpinSystem::initialize(unsigned short i, unsigned short entityID) {
    getSpin(i)->speed = 0.01f;
    getSpin(i)->spinAmount = 0.0f;
}

void SpinSystem::update(unsigned short i, unsigned short entityID) {
    Spin* s = getSpin(i);
    s->spinAmount += s->speed;
    EntityManager::instance->entities[entityID]->transform->rotation.setRotation(s->spinAmount, glm::vec3(0.0, 1.0, 0.0));
    //EntityManager::instance->entities[entityID]->transform->translation = glm::vec3(s->spinAmount, 0.0, 0.0);
}

void SpinSystem::addEntityComponent() {
    EntityManager::instance->spinComponents.push_back(Spin());
}

Spin* SpinSystem::getSpin(unsigned short i) {
    return &EntityManager::instance->spinComponents[i];
}