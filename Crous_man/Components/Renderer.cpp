#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <functional>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>
#include <common/texture.hpp>

#include "../ECS/EntityManager.hpp"
#include "../ECS/Bitmap.hpp"
#include "../Transform.hpp"
#include "../ECS/Entity.hpp"
#include "../Shaders/GShaders/TextureGShader.hpp"

#include "../Util.hpp"

#include "Renderer.hpp"

#include "../Shaders/GShader.hpp"
#include "../Shaders/MeshEShader.hpp"
#include "Mesh.hpp"


Renderer::Renderer() {
    this->draw = true;
    this->meshID = 0;
    this->gShaderInstance = NULL;

    this->vertexArray = 0;

    this->vertexbuffer = 0;
    this->textCoordBuffer = 0;
    this->normalsbuffer = 0;
    this->elementbuffer = 0;

    this->diffuseBuffer = loadTextureFromColor(glm::vec3(.408, .087, .915));
    this->specularBuffer = loadTextureFromFloat(1.0);
}

void Renderer::setDiffuseBuffer(GLuint diffuseBuffer) {
    deleteTextureBuffer(this->diffuseBuffer);
    this->diffuseBuffer = diffuseBuffer;
}

void Renderer::setSpecularBuffer(GLuint specularBuffer) {
    deleteTextureBuffer(this->specularBuffer);
    this->specularBuffer = specularBuffer;
}


RendererSystem::RendererSystem() : ComponentSystem(){
    requiredComponentsBitmap = new Bitmap({SystemIDs::RendererID, SystemIDs::MeshID});
}

RendererSystem::~RendererSystem() {

}

void RendererSystem::initialize(unsigned short i, unsigned short entityID) {
    Renderer* r = getRenderer(i);
    if (r->gShaderInstance == NULL) {
        r->gShaderInstance = TextureGShader::instance;

        r->meshID = EntityManager::instance->getComponentId(SystemIDs::MeshID, entityID);
    }
}

void RendererSystem::initBuffers(unsigned short i, unsigned short entityID) {
    Renderer* r = getRenderer(i);
    if (r->gShaderInstance == NULL) {
        initialize(i, entityID);
    }
    Mesh* m = getMesh(r->meshID);

    r->gShaderInstance->use();

    glGenVertexArrays(1, &r->vertexArray);
    glBindVertexArray(r->vertexArray);


    glGenBuffers(1, &r->vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, r->vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, m->indexed_vertices.size() * sizeof(glm::vec3), &m->indexed_vertices[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, r->vertexbuffer);
    glVertexAttribPointer(
                0,                  // attribute
                3,                  // size
                GL_FLOAT,           // type
                GL_FALSE,           // normalized?
                0,                  // stride
                (void*)0            // array buffer offset
    );



    glGenBuffers(1, &r->normalsbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, r->normalsbuffer);
    glBufferData(GL_ARRAY_BUFFER, m->normals.size() * sizeof(glm::vec3), &m->normals[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, r->normalsbuffer);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);



    glGenBuffers(1, &r->textCoordBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, r->textCoordBuffer);
    glBufferData(GL_ARRAY_BUFFER, m->UV.size() * sizeof(glm::vec2), &m->UV[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(2);
    glBindBuffer(GL_ARRAY_BUFFER, r->textCoordBuffer);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);



    glGenBuffers(1, &r->elementbuffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, r->elementbuffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m->indices.size() * sizeof(unsigned short), &m->indices[0] , GL_STATIC_DRAW);



    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void RendererSystem::addEntityComponent() {
    EntityManager::instance->rendererComponents.push_back(Renderer());
}

void RendererSystem::renderAll(glm::mat4 view, glm::mat4 projection) {
    unsigned short entityID;
    
    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        entityID = entityIDs[i];
        if (!EntityManager::instance->shouldUpdate(entityIDs[i]))
            continue;

        Renderer* r = getRenderer(i);
        Mesh* m = getMesh(r->meshID);
        GShader* gS = r->gShaderInstance;
        Entity* e = EntityManager::instance->entities[entityID];

        if (!r->draw) {
            continue;
        }
        
        gS->use();
        gS->prerender(r);
    
        glm::mat4 model = e->worldTransform->toMat4();
        glm::mat4 normal = e->worldTransform->toNormal();

        gS->setMVPN(model, view, projection, normal);

        glBindVertexArray(r->vertexArray);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        
        glDrawElements(
                    GL_TRIANGLES,      // mode
                    m->indices.size(), // count
                    GL_UNSIGNED_SHORT, // type
                    (void*)0           // element array buffer offset
        );

        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        glDisableVertexAttribArray(2);

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    std::cout << std::endl;
}

void RendererSystem::renderUsingShader(MeshEShader* shader, glm::mat4 view, glm::mat4 projection) {
    unsigned short entityID;

    shader->use();

    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        entityID = entityIDs[i];
        if (!EntityManager::instance->shouldUpdate(entityIDs[i]))
            continue;

        Renderer* r = getRenderer(i);
        Mesh* m = getMesh(r->meshID);
        Entity* e = EntityManager::instance->entities[entityID];

        if (!shader->acceptRendererIf(r)) {
            continue;
        }

        glm::mat4 model = e->worldTransform->toMat4();
        glm::mat4 normal = e->worldTransform->toNormal();

        shader->setMVPN(model, view, projection, normal);

        glBindVertexArray(r->vertexArray);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);

        glDrawElements(
            GL_TRIANGLES,      // mode
            m->indices.size(), // count
            GL_UNSIGNED_SHORT, // type
            (void*)0           // element array buffer offset
        );

        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        glDisableVertexAttribArray(2);

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }
}

Renderer* RendererSystem::getRenderer(unsigned short i) {
    return &EntityManager::instance->rendererComponents[i];
}

Mesh* RendererSystem::getMesh(unsigned short i) {
    return &EntityManager::instance->meshComponents[i];
}