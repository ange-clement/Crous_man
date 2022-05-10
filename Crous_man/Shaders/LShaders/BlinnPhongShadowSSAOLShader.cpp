#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "../../Components/Camera.hpp"

#include "BlinnPhongShadowSSAOLShader.hpp"

BlinnPhongShadowSSAOLShader* BlinnPhongShadowSSAOLShader::instance = NULL;

BlinnPhongShadowSSAOLShader::BlinnPhongShadowSSAOLShader() : LShader("Shaders/LShaders/BlinnPhongShadowSSAOLShader.glsl") {
    if (BlinnPhongShadowSSAOLShader::instance == NULL) {
        BlinnPhongShadowSSAOLShader::instance = this;

        this->gShadow = glGetUniformLocation(this->programID, "gShadow");
        this->gSSAO = glGetUniformLocation(this->programID, "gSSAO");
        this->ambiantLocation = glGetUniformLocation(this->programID, "ambiant");

        glUniform1i(this->gShadow, 3);
        glUniform1i(this->gSSAO, 4);
    }
    else {
        std::cerr << "Error : cannot instanciate two BlinnPhongShadowSSAOLShader" << std::endl;
    }
}

BlinnPhongShadowSSAOLShader::~BlinnPhongShadowSSAOLShader() {

}

void BlinnPhongShadowSSAOLShader::setAmbiant(float ambiant) {
    glUniform1f(this->ambiantLocation, ambiant);
}

void BlinnPhongShadowSSAOLShader::useBuffers(const GLuint* buffers) {
    LShader::useBuffers(buffers);

    glUniform1i(this->gShadow, 3);
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, buffers[RenderBufferID::Shadow]);
    
    glUniform1i(this->gSSAO, 4);
    glActiveTexture(GL_TEXTURE4);
    glBindTexture(GL_TEXTURE_2D, buffers[RenderBufferID::SSAO]);
}