#ifndef BLUR_QUAD_E_SHADER_HPP
#define BLUR_QUAD_E_SHADER_HPP

#include "../QuadEShader.hpp"

class BlurQuadEShader : public QuadEShader {
public:

    unsigned short bufferIn  = (unsigned short) -1;
    unsigned short bufferOut = (unsigned short) -1;

    GLuint uTextureBufferLocation;

public:
    BlurQuadEShader(unsigned short bufferIn, unsigned short bufferOut);
    ~BlurQuadEShader();

    virtual void useBuffers(const GLuint* buffers);
    virtual void setOutputShaders(GLuint* buffers);
};

#endif