#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "TextureGShader.hpp"

TextureGShader* TextureGShader::instance = NULL;

TextureGShader::TextureGShader() : GShader("Shaders/TextureGShader.glsl") {
    if (TextureGShader::instance == NULL) {
        TextureGShader::instance = this;
    } else {
        std::cerr << "Error : cannot instanciate two TextureGShader" << std::endl;
    }
}

TextureGShader::~TextureGShader() {

}