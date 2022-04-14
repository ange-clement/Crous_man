#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "textureFramebuffer.hpp"

TextureFramebuffer::TextureFramebuffer() {

}

TextureFramebuffer::TextureFramebuffer(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT) {
    init(SCR_WIDTH, SCR_HEIGHT);
}

void TextureFramebuffer::update(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT) {
    glDeleteTextures(1, &textureColorBuffer);
    glDeleteFramebuffers(1, &framebuffer);
    init(SCR_WIDTH, SCR_HEIGHT);
}

void TextureFramebuffer::init(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT) {
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

    // position color buffer
    glGenTextures(1, &textureColorBuffer);
    glBindTexture(GL_TEXTURE_2D, textureColorBuffer);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, SCR_WIDTH, SCR_HEIGHT, 0, GL_RGB, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureColorBuffer, 0);
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

void TextureFramebuffer::use() {
    glBindFramebuffer(GL_FRAMEBUFFER, this->framebuffer);
}