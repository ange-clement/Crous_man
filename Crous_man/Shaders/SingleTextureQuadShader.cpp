#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "SingleTextureQuadShader.hpp"

SingleTextureQuadShader* SingleTextureQuadShader::instance = NULL;

SingleTextureQuadShader::SingleTextureQuadShader() : PEShader("Shaders/SingleTextureQuadShader.glsl") {
    if (SingleTextureQuadShader::instance == NULL) {
        SingleTextureQuadShader::instance = this;
    } else {
        std::cerr << "Error : cannot instanciate two SingleTextureQuadShader" << std::endl;
    }
}

SingleTextureQuadShader::~SingleTextureQuadShader() {

}