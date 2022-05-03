#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "ColliderShader.hpp"



ColliderShader::ColliderShader() : Shader("Shaders/ColliderVertexShader.glsl", "Shaders/ColliderFragmentShader.glsl") {
	this->colliderColor = glGetUniformLocation(this->programID, "u_color");
    this->model = glGetUniformLocation(this->programID, "u_modMat");
    this->view = glGetUniformLocation(this->programID, "u_viewMat");
    this->projection = glGetUniformLocation(this->programID, "u_projMat");
}

ColliderShader::~ColliderShader() {}

void ColliderShader::setMVP(const glm::mat4& model, const glm::mat4& view, const glm::mat4& projection) {
    glUniformMatrix4fv(this->model, 1, GL_FALSE, &model[0][0]);
    glUniformMatrix4fv(this->view, 1, GL_FALSE, &view[0][0]);
    glUniformMatrix4fv(this->projection, 1, GL_FALSE, &projection[0][0]);
}