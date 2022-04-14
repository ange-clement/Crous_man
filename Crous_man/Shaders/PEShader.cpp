#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "PEShader.hpp"

PEShader::PEShader(std::string fs) : QuadShader(fs) {

}

PEShader::~PEShader() {

}

void PEShader::use() {
    QuadShader::use();
}

void PEShader::setColorTexture(GLuint colorTexture) {
    glUniform1i(this->colorTexture, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, colorTexture);
}