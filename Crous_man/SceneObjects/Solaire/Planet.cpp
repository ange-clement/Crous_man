#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "../../../common/meshGenerator.hpp"

#include "../../Util.hpp"
#include "../../../external/image_ppm.h"
#include "../../Scene.hpp"

#include "Planet.hpp"

#include "../../Transform.hpp"
#include "../../Mesh.hpp"

Planet::Planet(std::string texture, std::string vertexShader, std::string fragmentShader) : SceneObject() {
    uvSphere(this->mesh->indexed_vertices, this->mesh->normals, this->UV, this->mesh->indices, this->mesh->triangles,
                0.5, 64, 32);

    //openOFF("../ressources/sphere.off", this->mesh->indexed_vertices, this->mesh->normals, this->mesh->indices, this->mesh->triangles, true);
    //this->mesh->computeTrianglesNormals();
    //computeUV();
    this->mesh->useProgram();

    glGenVertexArrays(1, &this->mesh->VertexArrayID);
    glBindVertexArray(this->mesh->VertexArrayID);

    glGenBuffers(1, &this->mesh->vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, this->mesh->vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, this->mesh->indexed_vertices.size() * sizeof(glm::vec3), &this->mesh->indexed_vertices[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, this->mesh->vertexbuffer);
    glVertexAttribPointer(
                0,                  // attribute
                3,                  // size
                GL_FLOAT,           // type
                GL_FALSE,           // normalized?
                0,                  // stride
                (void*)0            // array buffer offset
    );

    glGenBuffers(1, &this->mesh->normalsbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, this->mesh->normalsbuffer);
    glBufferData(GL_ARRAY_BUFFER, this->mesh->normals.size() * sizeof(glm::vec3), &this->mesh->normals[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, this->mesh->normalsbuffer);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);




    glGenBuffers(1, &this->UVBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, this->UVBuffer);
    glBufferData(GL_ARRAY_BUFFER, this->UV.size() * sizeof(glm::vec2), &this->UV[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(2);
    glBindBuffer(GL_ARRAY_BUFFER, this->UVBuffer);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);




    glGenBuffers(1, &this->mesh->elementbuffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->mesh->elementbuffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, this->mesh->indices.size() * sizeof(unsigned short), &this->mesh->indices[0] , GL_STATIC_DRAW);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);


    this->mesh->loadShaders(vertexShader, fragmentShader);

    this->texture = new Texture();
    this->texture->loadTexture(texture.c_str());

    glUniform1i(glGetUniformLocation(this->mesh->programID, "texture"), 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, this->texture->texture);

    glUseProgram(0);
}

void Planet::computeUV() {
    size_t size = this->mesh->indexed_vertices.size();
    this->UV.resize(size);
    srand(time(NULL));
    for (size_t i = 0; i < size; i++) {
        this->UV[i] = glm::vec2(rand() /(float) RAND_MAX, rand() /(float) RAND_MAX);
    }
}

void Planet::draw(glm::mat4 view, glm::mat4 projection) {
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, this->texture->texture);
    SceneObject::draw(view, projection);
}