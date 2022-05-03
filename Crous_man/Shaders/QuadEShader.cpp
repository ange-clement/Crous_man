#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "QuadEShader.hpp"

#include <common/frameBuffer.hpp>

QuadEShader::QuadEShader(std::string fs, unsigned int width, unsigned int height, unsigned int numberOfBuffers) : QuadShader(fs) {
	fBuffer = new FrameBuffer(width, height, numberOfBuffers);
}

QuadEShader::~QuadEShader() {

}

void QuadEShader::updateBufferWidthHeight(unsigned int width, unsigned int height) {
	fBuffer->update(width, height);
}