#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "frameBuffer.hpp"

FrameBuffer::FrameBuffer() {

}

FrameBuffer::FrameBuffer(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT, unsigned int numberOfBuffer) {
    this->numberOfBuffer = numberOfBuffer;
    init(SCR_WIDTH, SCR_HEIGHT);
}

void FrameBuffer::update(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT) {
    for (unsigned int i = 0; i < numberOfBuffer; i++) {
        glDeleteTextures(1, &buffers[i]);
    }
    glDeleteFramebuffers(1, &frameBufferObject);
    init(SCR_WIDTH, SCR_HEIGHT);
}

void FrameBuffer::init(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT) {
    glGenFramebuffers(1, &frameBufferObject);
    glBindFramebuffer(GL_FRAMEBUFFER, frameBufferObject);

    unsigned int* attachments = new unsigned int[this->numberOfBuffer];

    for (unsigned int i = 0; i < this->numberOfBuffer; i++) {
        glGenTextures(1, &buffers[i]);
        glBindTexture(GL_TEXTURE_2D, buffers[i]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, SCR_WIDTH, SCR_HEIGHT, 0, GL_RGBA, GL_FLOAT, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + i, GL_TEXTURE_2D, buffers[i], 0);
        attachments[i] = GL_COLOR_ATTACHMENT0 + i;
    }
    // tell OpenGL which color attachments we'll use (of this framebuffer) for rendering 
    glDrawBuffers(this->numberOfBuffer, attachments);

    // create and attach depth buffer (renderbuffer)
    glGenRenderbuffers(1, &rboDepth);
    glBindRenderbuffer(GL_RENDERBUFFER, rboDepth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, SCR_WIDTH, SCR_HEIGHT);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rboDepth);
    // finally check if framebuffer is complete
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cout << "Framebuffer not complete!" << std::endl;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}