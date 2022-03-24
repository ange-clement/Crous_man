#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "Util.hpp"

#include "Scene.hpp"

#include "Transform.hpp"
#include "Mesh.hpp"
#include "SceneObjects/PointLight.hpp"
#include "InputManager.hpp"

SceneObject::SceneObject() {
    transform = new Transform();
    worldTransform = new Transform();
    mesh = new Mesh();
    parent = NULL;
}

SceneObject::~SceneObject() {
    delete transform;
    delete worldTransform;
    delete mesh;

    childrens.clear();
}

void SceneObject::addChildren(SceneObject* children) {
    this->childrens.push_back(children);
    children->parent = this;
}

void SceneObject::applyTransformToPoints() {
    for (size_t i = 0, size = mesh->indexed_vertices.size(); i < size; i++) {
        mesh->indexed_vertices[i] = transform->applyToPoint(mesh->indexed_vertices[i]);
    }
}

void SceneObject::loadMesh(std::string fileName, bool fileHasNormals) {
    mesh->loadMesh(fileName, fileHasNormals);
}

void SceneObject::loadMesh(std::string fileName) {
    mesh->loadMesh(fileName, false);
}

void SceneObject::loadShaders() {
    mesh->loadShaders();
}

glm::vec3 SceneObject::localToWorld(glm::vec3 localposition) {
    return worldTransform->applyToPoint(localposition);
}

void SceneObject::processInput(GLFWwindow *window, InputManager* inputManager, float deltaTime) {
    for (size_t i = 0, size = this->childrens.size(); i < size; i++) {
        this->childrens[i]->processInput(window, inputManager, deltaTime);
    }
}

void SceneObject::updateTransforms() {
    Transform* parentTransform;
    if (parent != NULL) {
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


void SceneObject::updateLights(const std::vector<PointLight*> & pointLights) {
    this->mesh->updateLights(pointLights);
    for (size_t i = 0, size = this->childrens.size(); i < size; i++) {
        this->childrens[i]->updateLights(pointLights);
    }
}

void SceneObject::draw(glm::mat4 view, glm::mat4 projection) {
    this->mesh->draw(worldTransform, view, projection);
    for (size_t i = 0, size = this->childrens.size(); i < size; i++) {
        this->childrens[i]->draw(view, projection);
    }
}