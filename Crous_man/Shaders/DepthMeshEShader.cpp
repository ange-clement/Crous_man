#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "DepthMeshEShader.hpp"

DepthMeshEShader* DepthMeshEShader::instance = NULL;

DepthMeshEShader::DepthMeshEShader() : MeshEShader(
    "Shaders/DepthMeshEShader.glsl",
    800, 600, 1,
    [](Renderer* r) { return true; } )
{
    if (DepthMeshEShader::instance == NULL) {
        DepthMeshEShader::instance = this;

        this->uFromPosLocation = glGetUniformLocation(this->programID, "uFromPos");

        glUniform1i(this->uFromPosLocation, 0);
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