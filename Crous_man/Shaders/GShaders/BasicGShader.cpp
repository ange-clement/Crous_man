#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "BasicGShader.hpp"

BasicGShader* BasicGShader::instance = NULL;

BasicGShader::BasicGShader() : GShader("Shaders/GShaders/BasicGShader.glsl") {
    if (BasicGShader::instance == NULL) {
        BasicGShader::instance = this;
    } else {
        std::cerr << "Error : cannot instanciate two BasicGShader" << std::endl;
    }
}

BasicGShader::~BasicGShader() {

}