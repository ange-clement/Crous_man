#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <common/shader.hpp>

#include "Shader.hpp"

Shader::Shader(std::string vs, std::string fs) {
    this->programID = LoadShaders(vs.c_str(), fs.c_str());
}

Shader::~Shader() {
    glDeleteProgram(this->programID);
}

void Shader::use() {
    glUseProgram(this->programID);
}