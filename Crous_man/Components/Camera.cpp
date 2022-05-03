#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>


#include <common/basicShapeRender.hpp>

#include "../Util.hpp"

#include "../ECS/EntityManager.hpp"
#include "../ECS/Bitmap.hpp"
#include "../ECS/Entity.hpp"
#include "../Transform.hpp"

#include "../Shaders/BlinnPhongLShader.hpp"
#include "../Shaders/SingleTextureQuadShader.hpp"

#include "Camera.hpp"

#include "Renderer.hpp"
#include "PointLight.hpp"
#include "Collider.hpp"
#include <common/gBuffer.hpp>

#include "../Shaders/LShader.hpp"
#include "../Shaders/PEShader.hpp"

Camera::Camera() {
    
}

void Camera::updateWidthHeight(unsigned int width, unsigned int height) {
    this->SCR_WIDTH  = width;
    this->SCR_HEIGHT = height;
    this->fov = width /(float) height;
    this->gBuffer.update(width, height);
    this->textureFramebuffer.update(width, height);
}

CameraSystem::CameraSystem() : ComponentSystem(){
    requiredComponentsBitmap =  new Bitmap({SystemIDs::CameraID});
    rendererInstance =          dynamic_cast<RendererSystem*>(EntityManager::instance->systems[SystemIDs::RendererID]);
    pointLightInstance =        dynamic_cast<PointLightSystem*>(EntityManager::instance->systems[SystemIDs::PointLightID]);
    colliderRenderInstance =    dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID]);
}

CameraSystem::~CameraSystem() {

}

void CameraSystem::initialize(unsigned short i, unsigned short entityID) {
    Camera* c = getCamera(i);
    c->lShaderInstance = BlinnPhongLShader::instance;
    c->peShaderInstance = SingleTextureQuadShader::instance;
    c->gBuffer = GBuffer(c->SCR_WIDTH, c->SCR_HEIGHT);
    c->textureFramebuffer = TextureFramebuffer(c->SCR_WIDTH, c->SCR_HEIGHT);
}

void CameraSystem::render(unsigned short i, unsigned short entityID) {
    Entity* e = EntityManager::instance->entities[entityID];
    Camera* c = getCamera(i);

    glm::mat4 view = glm::lookAt(
        e->worldTransform->translation,
        e->worldTransform->applyToPoint(glm::vec3(0.0, 0.0, 1.0)),
        glm::vec3(0.0, 1.0, 0.0)
    );
    //glm::mat4 view = e->worldTransform->inverse()->toMat4();

    glm::mat4 projection = glm::perspective(
        c->fov,
        c->ratio,
        c->minRange, c->maxRange
    );

    // 1. geometry pass: render scene's geometry/color data into gbuffer
    // -----------------------------------------------------------------
    glBindFramebuffer(GL_FRAMEBUFFER, c->gBuffer.gBuffer);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    rendererInstance->renderAll(view, projection);


    c->textureFramebuffer.use();
    // 2. lighting pass: calculate lighting by iterating over a screen filled quad pixel-by-pixel using the gbuffer's content.
    // -----------------------------------------------------------------------------------------------------------------------
    LShader* ls = c->lShaderInstance;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    ls->use();
    ls->setViewPos(e->worldTransform->translation);
    ls->setBuffers(c->gBuffer.gPosition, c->gBuffer.gNormal, c->gBuffer.gAlbedoSpec);

    // send light relevant uniforms
    std::vector<PointLight>* lights = &EntityManager::instance->pointLightComponents;
    Entity* lightEntity;
    PointLight* l;
    for (unsigned int i = 0; i < lights->size(); i++)
    {
        if (pointLightInstance->entityIDs[i] == (unsigned short)-1)
            continue;
        lightEntity = EntityManager::instance->entities[pointLightInstance->entityIDs[i]];
        l = &(*lights)[i];
        ls->setLight(i, lightEntity->worldTransform->translation, l->color, l->linear, l->quadratic);
    }
    // finally render quad
    BasicShapeRender::instance->renderQuad();

    //TODO bind final framebuffer ( 0 iff screen )
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    // 3. post effect pass: do something with the image then render it
    // ---------------------------------------------------------------
    //TODO : plusieurs PEShaders? (ping pong framebuffer ou écrire dans une texture?)
    PEShader* pe = c->peShaderInstance;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    pe->use();
    pe->setColorTexture(c->textureFramebuffer.textureColorBuffer);
    BasicShapeRender::instance->renderQuad();

    //Now render basic colliders
    colliderRenderInstance->renderAll(view, projection);
}

void CameraSystem::addEntityComponent() {
    EntityManager::instance->cameraComponents.push_back(Camera());
}

void CameraSystem::setScreenCamera(unsigned short entityID) {
    this->screenCamera = getCamera(getComponentId(entityID));
}

Camera* CameraSystem::getCamera(unsigned short i) {
    return &EntityManager::instance->cameraComponents[i];
}