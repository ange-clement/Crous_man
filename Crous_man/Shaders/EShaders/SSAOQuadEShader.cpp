#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <random>
#include <string>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <common/frameBuffer.hpp>
#include <Crous_man/Transform.hpp>
#include <Crous_man/Util.hpp>

#include "../../ECS/EntityManager.hpp"

#include "../../Components/Camera.hpp"

#include "SSAOQuadEShader.hpp"

#define NB_SAMPLING 32

float lerp(float a, float b, float f)
{
    return a + f * (b - a);
}

SSAOQuadEShader::SSAOQuadEShader() : QuadEShader(
    "Shaders/EShaders/SSAOQuadEShader.glsl",
    800, 600, 1)
{
    this->gPositionLocation = glGetUniformLocation(this->programID, "gPosition");
    this->gNormalLocation   = glGetUniformLocation(this->programID, "gNormal");
    this->texNoiseLocation  = glGetUniformLocation(this->programID, "texNoise");

    this->viewLocation = glGetUniformLocation(this->programID, "view");
    this->projectionLocation = glGetUniformLocation(this->programID, "projection");

    glUniform1i(this->gPositionLocation, 0);
    glUniform1i(this->gNormalLocation  , 1);
    glUniform1i(this->texNoiseLocation , 2);


    // generate sample kernel
    // ----------------------
    std::uniform_real_distribution<GLfloat> randomFloats(0.0, 1.0); // generates random floats between 0.0 and 1.0
    std::default_random_engine generator;
    unsigned int nbSampling = NB_SAMPLING;
    for (unsigned int i = 0; i < nbSampling; ++i)
    {
        glm::vec3 sample(randomFloats(generator) * 2.0 - 1.0, randomFloats(generator) * 2.0 - 1.0, randomFloats(generator));
        sample = glm::normalize(sample);
        sample *= randomFloats(generator);
        float scale = float(i) / (float)nbSampling;

        // scale samples s.t. they're more aligned to center of kernel
        scale = lerp(0.1f, 1.0f, scale * scale);
        sample *= scale;
        this->ssaoKernel.push_back(sample);
    }

    // generate noise texture
    // ----------------------
    std::vector<glm::vec3> ssaoNoise;
    for (unsigned int i = 0; i < 16; i++)
    {
        glm::vec3 noise(randomFloats(generator) * 2.0 - 1.0, randomFloats(generator) * 2.0 - 1.0, 0.0f); // rotate around z-axis (in tangent space)
        ssaoNoise.push_back(noise);
    }
    
    glGenTextures(1, &this->noiseTexture);
    glBindTexture(GL_TEXTURE_2D, this->noiseTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, 4, 4, 0, GL_RGB, GL_FLOAT, &ssaoNoise[0]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
}

SSAOQuadEShader::~SSAOQuadEShader() {
    glDeleteTextures(1, &this->noiseTexture);
}

void SSAOQuadEShader::useBuffers(const GLuint* buffers) {
    glUniform1i(this->gPositionLocation, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, buffers[RenderBufferID::Position]);

    glUniform1i(this->gNormalLocation, 1);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, buffers[RenderBufferID::Normal]);

    glUniform1i(this->texNoiseLocation, 2);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, this->noiseTexture);

    // Send kernel + rotation 
    for (unsigned int i = 0; i < NB_SAMPLING; ++i)
        glUniform3fv(glGetUniformLocation(this->programID, ("samples[" + std::to_string(i) + "]").c_str()), 1, &this->ssaoKernel[i][0]);
}

void SSAOQuadEShader::useVP(const glm::mat4& view, const glm::mat4& projection) {
    glUniformMatrix4fv(this->viewLocation, 1, GL_FALSE, &view[0][0]);
    glUniformMatrix4fv(this->projectionLocation, 1, GL_FALSE, &projection[0][0]);
}

void SSAOQuadEShader::setOutputShaders(GLuint* buffers) {
    buffers[RenderBufferID::SSAO] = fBuffer->buffers[0];
}