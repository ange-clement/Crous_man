#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <common/frameBuffer.hpp>

#include "../../ECS/EntityManager.hpp"

#include "../../Components/Camera.hpp"

#include "BlurQuadEShader.hpp"

BlurQuadEShader::BlurQuadEShader(unsigned short bufferIn, unsigned short bufferOut) : QuadEShader(
    "Shaders/EShaders/BlurQuadEShader.glsl",
    800, 600, 1)
{
    this->bufferIn = bufferIn;
    this->bufferOut = bufferOut;

    this->uTextureBufferLocation = glGetUniformLocation(this->programID, "uTextureBuffer");
    glUniform1i(this->uTextureBufferLocation, 0);
}

BlurQuadEShader::~BlurQuadEShader() {

}

void BlurQuadEShader::useBuffers(const GLuint* buffers) {
    glUniform1i(this->uTextureBufferLocation, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, buffers[bufferIn]);
}

void BlurQuadEShader::setOutputShaders(GLuint* buffers) {
    buffers[bufferOut] = fBuffer->buffers[0];
}