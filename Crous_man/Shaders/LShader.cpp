#include <stdio.h>
#include <stdlib.h>
#include <vector>
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
    glUniform1i(this->gPosition, 0);
    glUniform1i(this->gNormal  , 1);
    glUniform1i(this->gAlbedo  , 2);
}