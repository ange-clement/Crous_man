#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "ShadowQuadEShader.hpp"

ShadowQuadEShader* ShadowQuadEShader::instance = NULL;

ShadowQuadEShader::ShadowQuadEShader() : QuadEShader(
    "Shaders/EShaders/ShadowQuadEShader.glsl",
    800, 600, 1)
{
    if (ShadowQuadEShader::instance == NULL) {
        ShadowQuadEShader::instance = this;

        this->gPositionLocation = glGetUniformLocation(this->programID, "gPosition");
        this->uShadowMapLocation = glGetUniformLocation(this->programID, "uShadowMap");
        this->uLightSpaceMatrixLocation = glGetUniformLocation(this->programID, "uLightSpaceMatrix");

        glUniform1i(this->gPositionLocation, 0);
        glUniform1i(this->uShadowMapLocation, 1);
    }
    else {
        std::cerr << "Error : cannot instanciate two ShadowQuadEShader" << std::endl;
    }
}

ShadowQuadEShader::~ShadowQuadEShader() {

}

void ShadowQuadEShader::setPosition(GLuint gPosition) {
    glUniform1i(this->gPositionLocation, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, gPosition);
}

void ShadowQuadEShader::setShadowMap(GLuint shadowMap) {
    glUniform1i(this->uShadowMapLocation, 1);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, shadowMap);
}

void ShadowQuadEShader::setLightSpaceMatrix(glm::mat4 lightSpaceMatrix) {
    glUniformMatrix4fv(this->uLightSpaceMatrixLocation, 1, false, &lightSpaceMatrix[0][0]);
}

void ShadowQuadEShader::useBuffers(std::vector<GLuint> buffers) {
    this->setPosition(buffers[0]);
    this->setShadowMap(buffers[3]);
}