#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "../../external/image_ppm.h"

#include "../Scene.hpp"
#include "LODObject.hpp"

#include "../Mesh.hpp"

LODObject::LODObject(std::vector<std::string> meshnames, std::vector<bool> hasNormal) : SceneObject() {
    this->numberOfLevels = meshnames.size();
    this->meshes.resize(meshnames.size());
    for (size_t i = 0; i < this->numberOfLevels; i++) {
        this->meshes[i] = new Mesh();
        this->meshes[i]->loadMesh(meshnames[i], hasNormal[i]);
        this->meshes[i]->loadShaders();
    }
}

void LODObject::setLevel(unsigned int l) {
    if (l < this->numberOfLevels) {
        this->mesh = this->meshes[l];
    }
    else {
       this->mesh = this->meshes[this->numberOfLevels-1];
    }
}