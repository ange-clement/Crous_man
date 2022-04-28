#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "LShader.hpp"

LShader::LShader(std::string fs) : QuadShader(fs) {
    this->gPosition = glGetUniformLocation(this->programID, "gPosition");
    this->gNormal   = glGetUniformLocation(this->programID, "gNormal");
    this->gAlbedo   = glGetUniformLocation(this->programID, "gAlbedo");
    //this->light     = glGetUniformLocation(this->programID, "light");
    this->viewPos   = glGetUniformLocation(this->programID, "viewPos");

    glUniform1i(this->gPosition, 0);
    glUniform1i(this->gNormal  , 1);
    glUniform1i(this->gAlbedo  , 2);
}

LShader::~LShader() {

}

void LShader::setViewPos(glm::vec3 viewPos) {
    glUniform3fv(this->viewPos, 1, &viewPos[0]);
}

void LShader::use() {
    QuadShader::use();
}

void LShader::setBuffers(GLuint gPosition, GLuint gNormal, GLuint gAlbedo) {
    glUniform1i(this->gPosition, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, gPosition);

    glUniform1i(this->gNormal  , 1);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, gNormal);

    glUniform1i(this->gAlbedo  , 2);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, gAlbedo);
}

void LShader::setLight(unsigned int index, glm::vec3 position, glm::vec3 color, float linear, float quadratic) {
    GLuint posPos       = glGetUniformLocation(this->programID, ("lights[" + std::to_string(index) + "].Position").c_str());
    GLuint colorPos     = glGetUniformLocation(this->programID, ("lights[" + std::to_string(index) + "].Color").c_str());
    GLuint linrearPos   = glGetUniformLocation(this->programID, ("lights[" + std::to_string(index) + "].Linear").c_str());
    GLuint quadraticPos = glGetUniformLocation(this->programID, ("lights[" + std::to_string(index) + "].Quadratic").c_str());

    glUniform3fv(posPos     , 1, &position[0]);
    glUniform3fv(colorPos   , 1, &color[0]);
    glUniform1f(linrearPos  , linear);
    glUniform1f(quadraticPos, quadratic);
}