#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>


#include <common/basicShapeRender.hpp>

#include "../ECS/EntityManager.hpp"
#include "../ECS/Bitmap.hpp"
#include "../ECS/Entity.hpp"
#include "../Transform.hpp"

#include "../Shaders/BlinnPhongLShader.hpp"

#include "Camera.hpp"

#include "Renderer.hpp"
#include <common/gBuffer.hpp>

#include "../Shaders/LShader.hpp"

CameraC::CameraC() {
    
}

CameraSystem::CameraSystem() : ComponentSystem(){
    requiredComponentsBitmap = new Bitmap({SystemIDs::CameraID});
    rendererInstance = dynamic_cast<RendererSystem*>(EntityManager::instance->systems[SystemIDs::RendererID]);
}

CameraSystem::~CameraSystem() {

}

void CameraSystem::initialize(unsigned short i, unsigned short entityID) {
    CameraC* c = getCamera(i);
    c->frameBufferObject = 0;
    c->lShaderInstance = BlinnPhongLShader::instance;
    c->gBuffer = GBuffer(c->SCR_WIDTH, c->SCR_HEIGHT);
}

void CameraSystem::update(unsigned short i, unsigned short entityID) {
    //TODO bind framebuffer et light?
    Entity* e = EntityManager::instance->entities[entityID];
    CameraC* c = getCamera(i);

    glm::mat4 view = glm::lookAt(
        e->worldTransform->translation,
        e->worldTransform->applyToPoint(glm::vec3(0.0, 0.0, 1.0)),
        glm::vec3(0.0, 1.0, 0.0)
    );
    //view = this->worldTransform->inverse()->toMat4();

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

    glBindFramebuffer(GL_FRAMEBUFFER, 0); // TODO bind framebuffer

    // 2. lighting pass: calculate lighting by iterating over a screen filled quad pixel-by-pixel using the gbuffer's content.
    // -----------------------------------------------------------------------------------------------------------------------
    LShader* ls = c->lShaderInstance;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    ls->use();
    ls->setViewPos(e->worldTransform->translation);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, c->gBuffer.gPosition);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, c->gBuffer.gNormal);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, c->gBuffer.gAlbedoSpec);
    // // TODO send light relevant uniforms
    // for (unsigned int i = 0; i < lightPositions.size(); i++)
    // {
    //     shaderLightingPass.setVec3("lights[" + std::to_string(i) + "].Position", lightPositions[i]);
    //     shaderLightingPass.setVec3("lights[" + std::to_string(i) + "].Color", lightColors[i]);
    //     // update attenuation parameters and calculate radius
    //     const float linear = 0.7f;
    //     const float quadratic = 1.8f;
    //     shaderLightingPass.setFloat("lights[" + std::to_string(i) + "].Linear", linear);
    //     shaderLightingPass.setFloat("lights[" + std::to_string(i) + "].Quadratic", quadratic);
    // }
    // shaderLightingPass.setVec3("viewPos", camera.Position);
    // finally render quad
    BasicShapeRender::instance->renderQuad();
}

void CameraSystem::addEntityComponent() {
    EntityManager::instance->cameraComponents.push_back(CameraC());
}

CameraC* CameraSystem::getCamera(unsigned short i) {
    return &EntityManager::instance->cameraComponents[i];
}