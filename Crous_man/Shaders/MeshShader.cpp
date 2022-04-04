#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "MeshShader.hpp"

MeshShader::MeshShader(std::string fs) : Shader("Shaders/MeshVertexShader.glsl", fs) {
    this->model      = glGetUniformLocation(this->programID, "model");
    this->view       = glGetUniformLocation(this->programID, "view");
    this->projection = glGetUniformLocation(this->programID, "projection");
    this->normal     = glGetUniformLocation(this->programID, "normal");
}

MeshShader::~MeshShader() {

}

void MeshShader::setMVPN(const glm::mat4& model, const glm::mat4& view, const glm::mat4& projection, const glm::mat4& normal) {
    glUniformMatrix4fv(this->model,      1, GL_FALSE, &model[0][0]);
    glUniformMatrix4fv(this->view,       1, GL_FALSE, &view[0][0]);
    glUniformMatrix4fv(this->projection, 1, GL_FALSE, &projection[0][0]);
    glUniformMatrix4fv(this->normal,     1, GL_FALSE, &normal[0][0]);
}