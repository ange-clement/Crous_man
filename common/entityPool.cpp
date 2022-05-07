#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>
#include <Crous_man/ECS/EntityManager.hpp>

#include "entityPool.hpp"

#include <Crous_man/ECS/Entity.hpp>



EntityPool::EntityPool(std::vector<Entity*> entities) {
	this->maxNumberOfObjects = entities.size();
	this->pool = new Entity * [this->maxNumberOfObjects];
	this->currentInstancedObject = 0;
	for (unsigned int i = 0; i < this->maxNumberOfObjects; i++) {
		this->pool[i] = entities[i];
	}
}

EntityPool::~EntityPool() {

}

Entity* EntityPool::addEntity() {
	pool[currentInstancedObject]->isActive = true;
	Entity* e = pool[currentInstancedObject];
	currentInstancedObject = (currentInstancedObject+1) % maxNumberOfObjects;
	return e;
}
void EntityPool::addEntity(unsigned int number) {
	for (unsigned int i = 0; i < number; i++) {
		this->addEntity();
	}
}
void EntityPool::deleteEntity() {
	pool[currentInstancedObject]->isActive = false;
	currentInstancedObject = (currentInstancedObject - 1) % maxNumberOfObjects;
}
void EntityPool::deleteEntity(unsigned int number) {
	for (unsigned int i = 0; i < number; i++) {
		this->deleteEntity();
	}
}