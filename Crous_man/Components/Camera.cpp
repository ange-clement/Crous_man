#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>


#include <common/basicShapeRender.hpp>
#include <common/frameBuffer.hpp>

#include "../Util.hpp"

#include "../ECS/EntityManager.hpp"
#include "../ECS/Bitmap.hpp"
#include "../ECS/Entity.hpp"
#include "../Transform.hpp"

#include "../Shaders/EShaders/DepthMeshEShader.hpp"
#include "../Shaders/EShaders/ShadowQuadEShader.hpp"
#include "../Shaders/EShaders/BlurQuadEShader.hpp"
#include "../Shaders/EShaders/SSAOQuadEShader.hpp"
#include "../Shaders/LShaders/BlinnPhongLShader.hpp"
#include "../Shaders/LShaders/BlinnPhongShadowLShader.hpp"
#include "../Shaders/LShaders/BlinnPhongShadowSSAOLShader.hpp"
#include "../Shaders/PEShaders/SingleTextureQuadShader.hpp"

#include "Camera.hpp"

#include "Renderer.hpp"
#include "PointLight.hpp"
#include "Collider.hpp"
#include <common/gBuffer.hpp>

#include "../Shaders/MeshEShader.hpp"
#include "../Shaders/QuadEShader.hpp"
#include "../Shaders/LShader.hpp"
#include "../Shaders/PEShader.hpp"
#include <common/ray.hpp>
#include <Crous_man/InputManager.hpp>

Camera::Camera() {

}

void Camera::updateWidthHeight(unsigned int width, unsigned int height) {
    this->SCR_WIDTH  = width;
    this->SCR_HEIGHT = height;
    this->fov = width /(float) height;
    this->gBuffer.update(width, height);
    this->textureFramebuffer.update(width, height);

    /*
    for (unsigned int i = 0, size = this->quadEShadersinstances.size(); i < size; i++) {
        this->quadEShadersinstances[i]->updateBufferWidthHeight(width, height);
    }
    for (unsigned int i = 0, size = this->meshEShadersinstances.size(); i < size; i++) {
        this->meshEShadersinstances[i]->updateBufferWidthHeight(width, height);
    }
    */
}

CameraSystem::CameraSystem() : ComponentSystem(){
    requiredComponentsBitmap =  new Bitmap({SystemIDs::CameraID});
    rendererInstance =          dynamic_cast<RendererSystem*>(EntityManager::instance->systems[SystemIDs::RendererID]);
    pointLightInstance =        dynamic_cast<PointLightSystem*>(EntityManager::instance->systems[SystemIDs::PointLightID]);
    colliderRenderInstance =    dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID]);

    buffers = new GLuint[RenderBufferID::NUMBER_OF_BUFFERS];
}

CameraSystem::~CameraSystem() {

}

void CameraSystem::initialize(unsigned short i, unsigned short entityID) {
    Camera* c = getCamera(i);

    ShadowQuadEShader* shadowEShader = ShadowQuadEShader::instance;
    shadowEShader->targetEntity = EntityManager::instance->entities[entityID];    

    c->quadEShadersinstances.push_back(shadowEShader);
    c->quadEShadersinstances.push_back(new SSAOQuadEShader());
    c->quadEShadersinstances.push_back(new BlurQuadEShader(RenderBufferID::SSAO, RenderBufferID::SSAO));
    c->quadEShadersinstances.push_back(new BlurQuadEShader(RenderBufferID::SSAO, RenderBufferID::SSAO));
    c->quadEShadersinstances.push_back(new BlurQuadEShader(RenderBufferID::SSAO, RenderBufferID::SSAO));
    c->lShaderInstance = BlinnPhongShadowSSAOLShader::instance;
    BlinnPhongShadowSSAOLShader::instance->use();
    BlinnPhongShadowSSAOLShader::instance->setAmbiant(0.1f);
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

    // 2. effect pass: Add new textures that may be used by the light shader
    // ---------------------------------------------------------------------
    //  2.1 mesh effect pass : render the scene meshes
    //  ----------------------------------------------
    
    buffers[RenderBufferID::Position] = c->gBuffer.gPosition;
    buffers[RenderBufferID::Normal] = c->gBuffer.gNormal;
    buffers[RenderBufferID::AlbedoSpec] = c->gBuffer.gAlbedoSpec;

    for (unsigned int i = 0, size = c->meshEShadersinstances.size(); i < size; i++) {
        // Render the meshes
        rendererInstance->renderUsingShader(c->meshEShadersinstances[i], view, projection);
        // Add the buffers to the list
        c->meshEShadersinstances[i]->setOutputShaders(buffers);
    }
    //  2.2 quad effect pass : render a quad and use previous buffers to generate
    //  -------------------------------------------------------------------------
    for (unsigned int i = 0, size = c->quadEShadersinstances.size(); i < size; i++) {
        // Render a quad
        c->quadEShadersinstances[i]->use();
        c->quadEShadersinstances[i]->useBuffers(buffers);
        c->quadEShadersinstances[i]->useVP(view, projection);
        BasicShapeRender::instance->renderQuad();
        // Add the buffers to the list
        c->quadEShadersinstances[i]->setOutputShaders(buffers);
    }

    c->textureFramebuffer.use();
    glViewport(0, 0, c->SCR_WIDTH, c->SCR_HEIGHT);
    // 3. lighting pass: calculate lighting by iterating over a screen filled quad pixel-by-pixel using the gbuffer's content.
    // -----------------------------------------------------------------------------------------------------------------------
    LShader* ls = c->lShaderInstance;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    ls->use();
    ls->setViewPos(e->worldTransform->translation);
    ls->useBuffers(buffers);

    // send light relevant uniforms
    std::vector<PointLight>* lights = &EntityManager::instance->pointLightComponents;
    Entity* lightEntity;
    PointLight* l;
    for (unsigned int i = 0; i < lights->size(); i++)
    {
        if (pointLightInstance->entityIDs[i] == (unsigned int) -1)
            continue;
        lightEntity = EntityManager::instance->entities[pointLightInstance->entityIDs[i]];
        if (lightEntity->isActive) {
            l = &(*lights)[i];
            ls->setLight(i, lightEntity->worldTransform->translation, l->color, l->linear, l->quadratic);
        }
        else {
            ls->setLight(i, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0f), 0.0f, 0.0f);
        }
    }
    // finally render quad
    BasicShapeRender::instance->renderQuad();

    //TODO bind final framebuffer ( 0 iff screen )
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    // 4. post effect pass: do something with the image then render it
    // ---------------------------------------------------------------
    //TODO : plusieurs PEShaders? (ping pong framebuffer ou ??crire dans une texture?)
    PEShader* pe = c->peShaderInstance;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    pe->use();
    pe->setColorTexture(c->textureFramebuffer.textureColorBuffer);
    BasicShapeRender::instance->renderQuad();

    //Now render basic colliders
    glClear(GL_DEPTH_BUFFER_BIT);
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


void intersectElements(Camera* c, glm::mat4 view, glm::mat4 projection) {
    //Perform LEFT clik behavior

    Ray ray = getPickRay(
        glm::vec2(InputManager::instance->lastMouseX, InputManager::instance->lastMouseY),
        glm::vec2(0),
        glm::vec2(c->SCR_WIDTH, c->SCR_HEIGHT),
        view, 
        projection
    );


    ColliderSystem* CS = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID]);
    //Now we can perform interactions with all colliders on a scene
    /*for (size_t i = 0, int size = EntityManager::instance->entities.size(); i < size; i++) {
        Collider* c = 0;
    }*/

}