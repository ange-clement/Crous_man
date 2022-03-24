#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>


#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <common/shader.hpp>
#include <common/objloader.hpp>

#include "Util.hpp"
#include "../external/image_ppm.h"
#include "Mesh.hpp"

#include "Transform.hpp"
#include "SceneObjects/PointLight.hpp"

Texture::Texture() {

}

unsigned int Texture::loadTextureGrey(const char* textureName) {  
    std::cout << "Loading texture : " << textureName << std::endl;      
    glGenTextures(1, &this->texture);
    glBindTexture(GL_TEXTURE_2D, this->texture);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    //load image
    int width, height;
    char cNomImgLue[250];
    sscanf (textureName,"%s",cNomImgLue);
    OCTET *ImgIn;

    lire_nb_lignes_colonnes_image_pgm(cNomImgLue, &height, &width);
    int nTaille = height * width;

    allocation_tableau(ImgIn, OCTET, nTaille);
    lire_image_pgm(cNomImgLue, ImgIn, nTaille);


    glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, width, height, 0, GL_RED, GL_UNSIGNED_BYTE, ImgIn);
    glGenerateMipmap(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);
    free(ImgIn);
    return this->texture;
}

    
unsigned int Texture::loadTexture(const char* textureName) {
    std::cout << "Loading texture : " << textureName << std::endl;
    glGenTextures(1, &this->texture);
    glBindTexture(GL_TEXTURE_2D, this->texture);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    //load image
    int width, height;
    char cNomImgLue[250];
    sscanf (textureName,"%s",cNomImgLue);
    OCTET *ImgIn;

    lire_nb_lignes_colonnes_image_ppm(cNomImgLue, &height, &width);
    int nTaille = height * width;
    int nTaille3 = nTaille * 3;

    allocation_tableau(ImgIn, OCTET, nTaille3);
    lire_image_ppm(cNomImgLue, ImgIn, nTaille);


    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, ImgIn);
    glGenerateMipmap(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);
    free(ImgIn);
    return this->texture;
}


Mesh::Mesh() {
    this->shaderLoaded = false;
}

Mesh::~Mesh() {
    // Cleanup VBO and shader
    glDeleteBuffers(1, &this->vertexbuffer);
    glDeleteBuffers(1, &this->normalsbuffer);
    glDeleteBuffers(1, &this->elementbuffer);
    glDeleteProgram(this->programID);
    glDeleteVertexArrays(1, &this->VertexArrayID);
}

void Mesh::loadShaders(std::string vertex_shader, std::string fragment_shader) {
    this->programID = LoadShaders(vertex_shader.c_str(), fragment_shader.c_str());
    useProgram();

    
    // Get a handle for our "Model View Projection" matrices uniforms
    this->uModelMatrixID      = glGetUniformLocation(this->programID, "uModelMatrix");
    this->uViewMatrixID       = glGetUniformLocation(this->programID, "uViewMatrix");
    this->uProjectionMatrixID = glGetUniformLocation(this->programID, "uProjectionMatrix");
    this->uNormalMatrixID     = glGetUniformLocation(this->programID, "uNormalMatrix");

    this->lightID = glGetUniformLocation(this->programID, "light_position_worldspace");
    this->numLightsID = glGetUniformLocation(this->programID, "numLights");

    this->shaderLoaded = true;
}

void Mesh::loadShaders() {
    this->loadShaders("shaders/default_vertex_shader.glsl", "shaders/default_fragment_shader.glsl");
}

void Mesh::loadMesh(std::string filename) {
    this->loadMesh(filename, false);
}

void Mesh::loadMesh(std::string filename, bool fileHasNormals) {

    if (fileHasNormals) {
        openOFF(filename, this->indexed_vertices, this->normals, this->indices, this->triangles, true);
    }
    else {
        loadOFF(filename, this->indexed_vertices, this->indices, this->triangles);
        computeNormals();
    }
    loadBuffers();
}

void Mesh::loadBuffers() {
    this->useProgram();

    glGenVertexArrays(1, &this->VertexArrayID);
    glBindVertexArray(this->VertexArrayID);

    glGenBuffers(1, &this->vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, this->vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, this->indexed_vertices.size() * sizeof(glm::vec3), &this->indexed_vertices[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, this->vertexbuffer);
    glVertexAttribPointer(
                0,                  // attribute
                3,                  // size
                GL_FLOAT,           // type
                GL_FALSE,           // normalized?
                0,                  // stride
                (void*)0            // array buffer offset
    );

    glGenBuffers(1, &this->normalsbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, this->normalsbuffer);
    glBufferData(GL_ARRAY_BUFFER, this->normals.size() * sizeof(glm::vec3), &this->normals[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, this->normalsbuffer);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

    glGenBuffers(1, &this->elementbuffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->elementbuffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, this->indices.size() * sizeof(unsigned short), &this->indices[0] , GL_STATIC_DRAW);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void Mesh::computeTrianglesNormals(){
    triangle_normals.clear();
    size_t size = triangles.size();
    triangle_normals.resize(size);
    for (size_t i = 0; i < size; i++) {
        glm::vec3 e_10 = indexed_vertices[triangles[i][1]] - indexed_vertices[triangles[i][0]];
        glm::vec3 e_20 = indexed_vertices[triangles[i][2]] - indexed_vertices[triangles[i][0]];

        triangle_normals[i] = glm::normalize(glm::cross(e_10, e_20));
    }
}

void Mesh::computeSmoothVertexNormal(){
    normals.clear();
    size_t size = indexed_vertices.size();
    normals.resize(size);

    for (size_t i = 0, triangleSize = triangles.size(); i < triangleSize; i++) {
        normals[triangles[i][0]] += triangle_normals[i];
        normals[triangles[i][1]] += triangle_normals[i];
        normals[triangles[i][2]] += triangle_normals[i];
    }

    for (size_t i = 0; i < size; i++) {
        normals[i] = glm::normalize(normals[i]);
    }
}

void Mesh::computeNormals() {
    computeTrianglesNormals();
    computeSmoothVertexNormal();
}

void Mesh::useProgram() {
    glUseProgram(this->programID);
}

void Mesh::updateLights(const std::vector<PointLight*> & pointLights) {
    if (!this->shaderLoaded) {
        return;
    }
    useProgram();
    
    int size = pointLights.size();
    float* positions = new float[MAX_LIGHTS*3];
    glm::vec3 v;
    int i;
    for (i = 0; i < size; i++) {
        v = pointLights[i]->worldTransform->translation;
        positions[3*i] = v[0];
        positions[3*i+1] = v[1];
        positions[3*i+2] = v[2];
    }
    glUniform3fv(this->lightID, MAX_LIGHTS, &positions[0]);
    glUniform1i(this->numLightsID, size);
}

void Mesh::draw(Transform* transform, glm::mat4 view, glm::mat4 projection) {
    if (!this->shaderLoaded) {
        return;
    }
    useProgram();
    glm::mat4 model = transform->toMat4();
    glm::mat4 normal = transform->toNormal();

    glUniformMatrix4fv(this->uModelMatrixID,      1, GL_FALSE, &model[0][0]);
    glUniformMatrix4fv(this->uViewMatrixID,       1, GL_FALSE, &view[0][0]);
    glUniformMatrix4fv(this->uProjectionMatrixID, 1, GL_FALSE, &projection[0][0]);
    glUniformMatrix4fv(this->uNormalMatrixID,     1, GL_FALSE, &normal[0][0]);

    glBindVertexArray(this->VertexArrayID);
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    glDrawElements(
                GL_TRIANGLES,      // mode
                indices.size(),    // count
                GL_UNSIGNED_SHORT, // type
                (void*)0           // element array buffer offset
    );

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}