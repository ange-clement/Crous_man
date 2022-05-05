#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "MeshEShader.hpp"

#include <common/frameBuffer.hpp>

MeshEShader::MeshEShader(std::string fs,
	unsigned int width, unsigned int height, unsigned int numberOfBuffers,
	std::function<bool(Renderer*)> acceptRendererIf) : MeshShader(fs)
{
	fBuffer = new FrameBuffer(width, height, numberOfBuffers);
	this->acceptRendererIf = acceptRendererIf;
}

MeshEShader::~MeshEShader() {

}

void MeshEShader::updateBufferWidthHeight(unsigned int width, unsigned int height) {
	fBuffer->update(width, height);
}

void MeshEShader::use() {
	MeshShader::use();
	this->fBuffer->use();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}