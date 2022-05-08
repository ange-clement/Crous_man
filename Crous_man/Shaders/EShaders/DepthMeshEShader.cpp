#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <Crous_man/Components/Renderer.hpp>

#include "DepthMeshEShader.hpp"

DepthMeshEShader* DepthMeshEShader::instance = NULL;

DepthMeshEShader::DepthMeshEShader() : MeshEShader(
    "Shaders/EShaders/DepthMeshEShader.glsl",
    1024, 1024, 1,
    [](Renderer* r) { return r->castShadows; } )
{
    if (DepthMeshEShader::instance == NULL) {
        DepthMeshEShader::instance = this;

        this->uFromPosLocation = glGetUniformLocation(this->programID, "uFromPos");
        this->uMaxDistanceLocation = glGetUniformLocation(this->programID, "uMaxDistance");
    }
    else {
        std::cerr << "Error : cannot instanciate two DepthMeshEShader" << std::endl;
    }
}

DepthMeshEShader::~DepthMeshEShader() {

}

void DepthMeshEShader::setFromPos(glm::vec3 fromPos) {
    glUniform3fv(this->uFromPosLocation, 1, &fromPos[0]);
}

void DepthMeshEShader::setMaxDistance(float maxDistance) {
    glUniform1f(this->uMaxDistanceLocation, maxDistance);
}