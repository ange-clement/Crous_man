#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "GShader.hpp"

#include "../Components/Renderer.hpp"

GShader::GShader(std::string fs) : MeshShader(fs) {

}

GShader::~GShader() {

}

void GShader::prerender(const Renderer* r) {

}