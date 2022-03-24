#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "../../external/image_ppm.h"

#include "../Scene.hpp"
#include "Terrain.hpp"

#include "../Transform.hpp"
#include "../Mesh.hpp"

Terrain::Terrain() : Terrain(glm::vec3(0.0, 0.0, 0.0), 1.0) {
    this->heightScale = 1.0/10.0;
    glUniform1f(this->heightScaleId, this->heightScale);
}

Terrain::Terrain(glm::vec3 start, float size) : SceneObject() {
    this->mesh->loadShaders("shaders/terrain_vertex_shader.glsl", "shaders/terrain_fragment_shader.glsl");
    this->mesh->useProgram();

    this->heightTexture = new Texture();
    this->heightTexture->loadTextureGrey("../ressources/heightmap256.pgm");

    //load image Height
    char cNomImgLue[250];
    sscanf ("../ressources/heightmap256.pgm","%s",cNomImgLue);

    lire_nb_lignes_colonnes_image_pgm(cNomImgLue, &height, &width);
    int nTaille = height * width;

    allocation_tableau(ImgHeight, OCTET, nTaille);
    lire_image_pgm(cNomImgLue, ImgHeight, nTaille);


    this->grassTexture = new Texture();
    this->grassTexture->loadTexture("../ressources/grass.ppm");

    this->rockTexture = new Texture();
    this->rockTexture->loadTexture("../ressources/rock.ppm");

    this->snowrockTexture = new Texture();
    this->snowrockTexture->loadTexture("../ressources/snowrock.ppm");


    glUniform1i(glGetUniformLocation(this->mesh->programID, "heightMap"), 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, this->heightTexture->texture);

    glUniform1i(glGetUniformLocation(this->mesh->programID, "grassTexture"), 1);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, this->grassTexture->texture);

    glUniform1i(glGetUniformLocation(this->mesh->programID, "rockTexture"), 2);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, this->rockTexture->texture);

    glUniform1i(glGetUniformLocation(this->mesh->programID, "snowrockTexture"), 3);
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, this->snowrockTexture->texture);

    this->heightScaleId = glGetUniformLocation(this->mesh->programID, "heightScale");
    this->heightTextureScaleId = glGetUniformLocation(this->mesh->programID, "heightTextureScale");
    this->textureScaleId = glGetUniformLocation(this->mesh->programID, "textureScale");

    glUniform1f(this->heightScaleId, this->heightScale);
    glUniform1f(this->heightTextureScaleId, this->heightTextureScale);
    glUniform1f(this->textureScaleId, this->textureScale);
    
    updatePlaneMesh(start, size);
}

void Terrain::updatePlaneMesh(glm::vec3 start, float size) {
    Mesh* mesh = this->mesh;
    mesh->indexed_vertices.clear();
    mesh->indices.clear();

    float increment = 1.0/(float)this->nbPointsWidth;
    for (size_t i = 0; i < this->nbPointsWidth; i++) {
        for (size_t j = 0; j < this->nbPointsWidth; j++) {
            mesh->indexed_vertices.push_back(start + glm::vec3(increment*j*size, 0.0, increment*i*size));
        }
    }
    this->applyTransformToPoints();
    glm::vec3 normal = glm::vec3(0.0, 1.0, 0.0);
    normal = this->transform->applyToVersor(normal);

    for (size_t i = 0; i < nbPointsWidth-1; i++) {
        for (size_t j = 0; j < nbPointsWidth-1; j++) {
            mesh->indices.push_back(i*nbPointsWidth+j);
            mesh->indices.push_back(i*nbPointsWidth+j+1);
            mesh->indices.push_back((i+1)*nbPointsWidth+j);

            mesh->indices.push_back((i+1)*nbPointsWidth+j);
            mesh->indices.push_back(i*nbPointsWidth+j+1);
            mesh->indices.push_back((i+1)*nbPointsWidth+j+1);
        }
    }

    mesh->computeNormals();
    mesh->loadBuffers();

    mesh->useProgram();

    glUniform3f(glGetUniformLocation(mesh->programID, "planeNormal"), normal.x, normal.y, normal.z);
    glUniform2f(glGetUniformLocation(mesh->programID, "planeHeightWidth"), size, size);
    glUniform3f(glGetUniformLocation(mesh->programID, "planeHeightStart"), start.x, start.y, start.z);
}


float Terrain::lerp(float a, float b, float v) {
    return a + (b-a)*v;
}

float Terrain::getHeight(glm::vec3 pointCoord) {
    glm::vec3 point = (glm::vec3(pointCoord) - this->transform->translation) / this->transform->scaling;
    point.x *= width;
    point.z *= height;
    if (    point.x <= 0 || point.z <= 0  ||
            point.x >= width ||
            point.z >= height )
    {
        return 0;
    }
    int pixelX = (int) point.x;
    int pixelY = (int) point.z;
    int pixelXC = (int) point.x+0.5;
    int pixelYC = (int) point.z+0.5;

    int ndg = ImgHeight[pixelY * width + pixelX];
    int ndgVH, ndgVV, ndgVD; // voisin horizontal/vertical/diagonal
    // voisin direction X / Y
    int VX = (point.x < pixelXC) ? -1 : 1;
    int VY = (point.y < pixelYC) ? -1 : 1;

    ndgVH = ImgHeight[pixelY * width + pixelX + VX];
    ndgVV = ImgHeight[(pixelY + VY) * width + pixelX];
    ndgVD = ImgHeight[(pixelY + VY) * width + pixelX + VX];

    float hauteurH, hauteurD;
    hauteurH = lerp(ndg, ndgVH, abs(point.x - pixelXC));
    hauteurD = lerp(ndgVV, ndgVD, abs(point.x - pixelXC));

    float hauteur = lerp(hauteurH, hauteurD, abs(point.z - pixelYC));
    
    return hauteur / 255.0 * this->transform->scaling.y * this->heightScale + this->transform->translation.y;
}

void Terrain::draw(glm::mat4 view, glm::mat4 projection) {

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, this->heightTexture->texture);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, this->grassTexture->texture);

    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, this->rockTexture->texture);

    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, this->snowrockTexture->texture);


    SceneObject::draw(view, projection);
}
