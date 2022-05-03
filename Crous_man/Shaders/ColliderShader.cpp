#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "ColliderShader.hpp"
#include <Crous_man/Components/Collider.hpp>


ColliderShader* ColliderShader::instance = NULL;

ColliderShader::ColliderShader() : Shader("Shaders/ColliderVertexShader.glsl", "Shaders/ColliderFragmentShader.glsl") {
	if(ColliderShader::instance == NULL){
		ColliderShader::instance = this;
		this->colliderColor = glGetUniformLocation(this->programID, "u_color");
		this->model = glGetUniformLocation(this->programID, "u_modMat");
		this->view = glGetUniformLocation(this->programID, "u_viewMat");
		this->projection = glGetUniformLocation(this->programID, "u_projMat");
	}
	else {
		std::cerr << "Error : cannot instanciate two ColliderShader" << std::endl;
	}	
}

ColliderShader::~ColliderShader() {}

void ColliderShader::setMVPC(const glm::mat4& model, const glm::mat4& view, const glm::mat4& projection, bool incontact) {
    glUniformMatrix4fv(this->model, 1, GL_FALSE, &model[0][0]);
    glUniformMatrix4fv(this->view, 1, GL_FALSE, &view[0][0]);
    glUniformMatrix4fv(this->projection, 1, GL_FALSE, &projection[0][0]);
	glUniform3fv(this->colliderColor, 1,((incontact)? &onColidColor[0] : &notOnColidColor[0]));
}

void ColliderShader::init(Collider* c) {
    instance->use();
	glGenVertexArrays(1, &c->VertexArrayID);
	glBindVertexArray(c->VertexArrayID);

	glGenBuffers(1, &c->vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, c->vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, c->vertices.size() * sizeof(glm::vec3), &c->vertices[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, c->vertexbuffer);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

	glGenBuffers(1, &c->elementbuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, c->elementbuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, c->indices.size() * sizeof(unsigned short), &c->indices[0], GL_STATIC_DRAW);

	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glUseProgram(0);
}

void ColliderShader::draw(Collider* c) {
	glLineWidth(10.0f);
	glBindVertexArray(c->VertexArrayID);
	glEnableVertexAttribArray(0);
	glDrawElements(GL_LINES, c->indices.size(), GL_UNSIGNED_SHORT, (void*)0);
	glDisableVertexAttribArray(0);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}


