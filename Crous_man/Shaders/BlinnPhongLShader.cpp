#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "BlinnPhongLShader.hpp"

BlinnPhongLShader* BlinnPhongLShader::instance = NULL;

BlinnPhongLShader::BlinnPhongLShader() : LShader("Shaders/BlinnPhongLShader.glsl") {
    if (BlinnPhongLShader::instance == NULL) {
        BlinnPhongLShader::instance = this;
    } else {
        std::cerr << "Error : cannot instanciate two BlinnPhongLShader" << std::endl;
    }
}

BlinnPhongLShader::~BlinnPhongLShader() {

}