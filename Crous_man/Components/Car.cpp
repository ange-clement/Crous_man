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
#include "../SoundManager.hpp"

#include "Car.hpp"

CarSystem::CarSystem() : ComponentSystem(){
    requiredComponentsBitmap = new Bitmap({SystemIDs::CarID});
}

CarSystem::~CarSystem() {

}

void CarSystem::initialize(unsigned short i, unsigned short entityID) {

}

void CarSystem::update(unsigned short i, unsigned short entityID) {
    Car* c = getCar(i);
    if (c->gowingToward >= c->carPathEntity->childrens.size()) {
        c->gowingToward = 0;
    }

    Entity* gowingTowardEntity = c->carPathEntity->childrens[c->gowingToward];
    
}

void CarSystem::addEntityComponent() {
    EntityManager::instance->CarComponents.push_back(Car());
}

Car* CarSystem::getCar(unsigned short i) {
    return &EntityManager::instance->CarComponents[i];
}