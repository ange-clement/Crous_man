#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "TextureGShader.hpp"
#include "../../Components/Renderer.hpp"

TextureGShader* TextureGShader::instance = NULL;

TextureGShader::TextureGShader() : GShader("Shaders/GShaders/TextureGShader.glsl") {
    if (TextureGShader::instance == NULL) {
        TextureGShader::instance = this;

        this->uDiffuseTexture  = glGetUniformLocation(this->programID, "uDiffuseTexture");
        this->uSpecularTexture = glGetUniformLocation(this->programID, "uSpecularTexture");

        glUniform1i(this->uDiffuseTexture  , 0);
        glUniform1i(this->uSpecularTexture , 1);
    } else {
        std::cerr << "Error : cannot instanciate two TextureGShader" << std::endl;
    }
}

TextureGShader::~TextureGShader() {

}

void TextureGShader::setBuffers(GLuint uDiffuseTexture, GLuint uSpecularTexture) {
    glUniform1i(this->uDiffuseTexture, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, uDiffuseTexture);

    glUniform1i(this->uSpecularTexture, 1);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, uSpecularTexture);
}

void TextureGShader::prerender(const Renderer* r) {
    this->setBuffers(r->diffuseBuffer, r->specularBuffer);
}