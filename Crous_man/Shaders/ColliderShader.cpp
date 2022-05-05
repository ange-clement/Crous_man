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
#include <Crous_man/Util.hpp>


ColliderShader::ColliderShader() : Shader("Shaders/ColliderVertexShader.glsl", "Shaders/ColliderFragmentShader.glsl") {}

ColliderShader::~ColliderShader() {
	glDeleteBuffers(1, &vertexbuffer);
	glDeleteBuffers(1, &elementbuffer);
	glDeleteVertexArrays(1, &VertexArrayID);

	vertices.clear();
	axis.clear();
	indices.clear();
}

void ColliderShader::setMVPC(const glm::mat4& model, const glm::mat4& view, const glm::mat4& projection, bool incontact,const glm::vec3& sizeCollider) {	
	glUniformMatrix4fv(this->model, 1, GL_FALSE, &model[0][0]);
    glUniformMatrix4fv(this->view, 1, GL_FALSE, &view[0][0]);
    glUniformMatrix4fv(this->projection, 1, GL_FALSE, &projection[0][0]);
	glUniform3fv(this->colliderColor, 1,((incontact)? &onColidColor[0] : &notOnColidColor[0]));
	glUniform3fv(this->size, 1, &sizeCollider[0]);
}

void ColliderShader::init() {
	init_geometry();
	use();
    glGenVertexArrays(1, &this->VertexArrayID);
	glBindVertexArray(this->VertexArrayID);

	glGenBuffers(1, &this->vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, this->vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, this->vertices.size() * sizeof(glm::vec3), &this->vertices[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, this->vertexbuffer);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

	glGenBuffers(1, &this->elementbuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->elementbuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, this->indices.size() * sizeof(unsigned short), &this->indices[0], GL_STATIC_DRAW);

	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glUseProgram(0);
}

void ColliderShader::draw() {
	glLineWidth(10.0f);
	glBindVertexArray(this->VertexArrayID);
	glEnableVertexAttribArray(0);
	glDrawElements(GL_LINES, this->indices.size(), GL_UNSIGNED_SHORT, (void*)0);
	glDisableVertexAttribArray(0);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void ColliderShader::init_uniform() {
	this->colliderColor = glGetUniformLocation(this->programID, "u_color");
	this->model = glGetUniformLocation(this->programID, "u_modMat");
	this->view = glGetUniformLocation(this->programID, "u_viewMat");
	this->projection = glGetUniformLocation(this->programID, "u_projMat");
	this->size = glGetUniformLocation(this->programID, "u_size");
}


SphereColliderShader* SphereColliderShader::instance = NULL;
SphereColliderShader::SphereColliderShader() : ColliderShader() {
	if (SphereColliderShader::instance == NULL) {
		SphereColliderShader::instance = this;
		init_uniform();
	}
	else {
		std::cerr << "Error : cannot instanciate two SphereColliderShader" << std::endl;
	}
}
SphereColliderShader::~SphereColliderShader() {

}
void SphereColliderShader::init_geometry() {
	//Init sphere geometry for drawing next
	vertices.clear();
	int num_segments = 100;

	for (int ii = 0; ii < num_segments; ii++) {
		float theta = 2.0f * 3.1415926f * float(ii) / float(num_segments);
		float x = cosf(theta);
		float y = sinf(theta);
		vertices.push_back(glm::vec3(x, y, 0));
	}
	for (int ii = 0; ii < num_segments; ii++) {
		float theta = 2.0f * 3.1415926f * float(ii) / float(num_segments);
		float z = cosf(theta);
		float x = sinf(theta);
		vertices.push_back(glm::vec3(x, 0, z));
	}

	for (int ii = 0; ii < num_segments; ii++) {
		float theta = 2.0f * 3.1415926f * float(ii) / float(num_segments);
		float z = cosf(theta);
		float y = sinf(theta);
		vertices.push_back(glm::vec3(0, y, z));
	}

	indices.clear();
	indices.push_back(0);
	for (size_t i = 0; i < num_segments; i++) {
		indices.push_back(i);
		indices.push_back(i);
	}
	indices.push_back(0);

	indices.push_back(num_segments);
	for (size_t i = num_segments; i < num_segments * 2; i++) {
		indices.push_back(i);
		indices.push_back(i);
	}
	indices.push_back(num_segments);

	indices.push_back(2 * num_segments);
	for (size_t i = 2 * num_segments; i < num_segments * 3; i++) {
		indices.push_back(i);
		indices.push_back(i);
	}
	indices.push_back(2 * num_segments);
}


BoxColliderShader* BoxColliderShader::instance = NULL;
BoxColliderShader::BoxColliderShader() : ColliderShader() {
	if (BoxColliderShader::instance == NULL) {
		BoxColliderShader::instance = this;
		init_uniform();
	}
	else {
		std::cerr << "Error : cannot instanciate two BoxColliderShader" << std::endl;
	}
}
BoxColliderShader::~BoxColliderShader() {

}
void BoxColliderShader::init_geometry() {
	//Init box geometry for drawing next
	vertices.resize(8);
	vertices[0] = glm::vec3(-1, -1, 1);
	vertices[1] = glm::vec3(1, -1, 1);
	vertices[2] = glm::vec3(1, 1, 1);
	vertices[3] = glm::vec3(-1, 1, 1);

	vertices[5] = glm::vec3(1, -1, -1);
	vertices[6] = glm::vec3(1, 1, -1);
	vertices[7] = glm::vec3(-1, 1, -1);
	vertices[4] = glm::vec3(-1, -1, -1);

	indices.resize(24);
	indices = {
		0, 1, 1, 2, 2, 3, 3, 0,
		4, 5, 5, 6, 6, 7, 7, 4,
		0, 4, 1, 5, 2, 6, 3, 7
	};
}
