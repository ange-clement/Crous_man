#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "QuadShader.hpp"

QuadShader::QuadShader(std::string fs) : Shader("Shaders/QuadVertexShader.glsl", fs) {

}

QuadShader::~QuadShader() {

}