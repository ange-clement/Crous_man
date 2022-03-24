#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "Entity.hpp"
#include "Bitmap.hpp"
#include "../Transform.hpp"

Entity::Entity() {
    componentsBitmap = new Bitmap();
    transform = new Transform();
    worldTransform = new Transform();
    parent = NULL;
}

Entity::Entity(std::initializer_list<SystemIDs> systems) : Entity() {
    componentsBitmap->loadFromSystemIDS(systems);
}

Entity::~Entity() {
    delete transform;
    delete worldTransform;

    childrens.clear();
}


void Entity::addChildren(Entity* children) {
    // delete it from it's parent childs
    if (children->parent != NULL) {
        for (size_t i = 0, size = children->parent->childrens.size(); i < size; i++) {
            if (children->parent->childrens[i] == children) {
                children->parent->childrens.erase(children->parent->childrens.begin()+i);
                break;
            }
        }
    }

    this->childrens.push_back(children);
    children->parent = this;
}

void Entity::updateTransforms() {
    Transform* parentTransform;
    if (this->parent != NULL) {
        parentTransform = parent->worldTransform;
    }
    else {
        parentTransform = new Transform();
    }
    
    delete this->worldTransform;
    this->worldTransform = parentTransform->combineWith(this->transform);

    for (size_t i = 0, size = this->childrens.size(); i < size; i++) {
        this->childrens[i]->updateTransforms();
    }
}