#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "../../Components/Camera.hpp"

#include "BlinnPhongShadowLShader.hpp"

BlinnPhongShadowLShader* BlinnPhongShadowLShader::instance = NULL;

BlinnPhongShadowLShader::BlinnPhongShadowLShader() : LShader("Shaders/LShaders/BlinnPhongShadowLShader.glsl") {
    if (BlinnPhongShadowLShader::instance == NULL) {
        BlinnPhongShadowLShader::instance = this;

        this->gShadow = glGetUniformLocation(this->programID, "gShadow");

        glUniform1i(this->gShadow, 3);
    }
    else {
        std::cerr << "Error : cannot instanciate two BlinnPhongShadowLShader" << std::endl;
    }
}

BlinnPhongShadowLShader::~BlinnPhongShadowLShader() {

}

void BlinnPhongShadowLShader::useBuffers(const GLuint* buffers) {
    LShader::useBuffers(buffers);

    glUniform1i(this->gShadow, 3);
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, buffers[RenderBufferID::Shadow]);
}