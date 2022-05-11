#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <common/objloader.hpp>

#include "../ECS/EntityManager.hpp"
#include "../ECS/Bitmap.hpp"

#include "Mesh.hpp"

void Mesh::loadFromFile(std::string filename, bool fileHasNormals) {
    if (fileHasNormals) {
        openOFF(filename, this->indexed_vertices, this->normals, this->indices, this->triangles, true);
    }
    else {
        loadOFF(filename, this->indexed_vertices, this->indices, this->triangles);
        computeNormals();
    }

    //TODO LOAD UV!!!
    size_t size = this->indexed_vertices.size();
    this->UV.resize(size);
    srand(time(NULL));
    for (size_t i = 0; i < size; i++) {
        this->UV[i] = glm::vec2(rand() /(float) RAND_MAX, rand() /(float) RAND_MAX);
    }
}

void Mesh::loadFromFilePLY(std::string filename, bool invertTriangles) {
    loadPLY(filename, this->indexed_vertices, this->normals, this->UV, this->indices, this->triangles, invertTriangles);
    computeTrianglesNormals();
}

void Mesh::computeTrianglesNormals(){
    triangle_normals.clear();
    size_t size = triangles.size();
    triangle_normals.resize(size);
    for (size_t i = 0; i < size; i++) {
        glm::vec3 e_10 = indexed_vertices[triangles[i][1]] - indexed_vertices[triangles[i][0]];
        glm::vec3 e_20 = indexed_vertices[triangles[i][2]] - indexed_vertices[triangles[i][0]];

        triangle_normals[i] = glm::normalize(glm::cross(e_10, e_20));
    }
}

void Mesh::computeSmoothVertexNormal(){
    normals.clear();
    size_t size = indexed_vertices.size();
    normals.resize(size);

    for (size_t i = 0, triangleSize = triangles.size(); i < triangleSize; i++) {
        normals[triangles[i][0]] += triangle_normals[i];
        normals[triangles[i][1]] += triangle_normals[i];
        normals[triangles[i][2]] += triangle_normals[i];
    }

    for (size_t i = 0; i < size; i++) {
        normals[i] = glm::normalize(normals[i]);
    }
}

void Mesh::computeNormals() {
    computeTrianglesNormals();
    computeSmoothVertexNormal();
}

void Mesh::invertNormals() {
    size_t size = triangle_normals.size();
    for (size_t i = 0; i < size; i++) {
        triangle_normals[i] = -triangle_normals[i];
    }

    size = normals.size();
    for (size_t i = 0; i < size; i++) {
        normals[i] = -normals[i];
    }

    size = triangles.size();
    unsigned short tmp;
    for (size_t i = 0; i < size; i++) {
        tmp = triangles[i][1];
        triangles[i][1] = triangles[i][2];
        triangles[i][2] = tmp;
    }

    size = indices.size();
    for (size_t i = 0; i < size; i+=3) {
        tmp = indices[i+1];
        indices[i+1] = indices[i+2];
        indices[i+2] = tmp;
    }
}



MeshSystem::MeshSystem() : ComponentSystem(){
    requiredComponentsBitmap = new Bitmap({SystemIDs::MeshID});
}

MeshSystem::~MeshSystem() {

}

void MeshSystem::initialize(unsigned short i, unsigned short entityID) {

}

void MeshSystem::addEntityComponent() {
    EntityManager::instance->meshComponents.push_back(Mesh());
}

Mesh* MeshSystem::getMesh(unsigned short i) {
    return &EntityManager::instance->meshComponents[i];
}