#ifndef QUAD_E_SHADER_HPP
#define QUAD_E_SHADER_HPP

#include "QuadShader.hpp"

class FrameBuffer;

class QuadEShader : public QuadShader {
public:
    FrameBuffer* fBuffer;
public:
    QuadEShader(std::string fs, unsigned int width, unsigned int height, unsigned int numberOfBuffers);
    ~QuadEShader();

    void updateBufferWidthHeight(unsigned int width, unsigned int height);

    virtual void useBuffers(std::vector<GLuint> buffers);

    virtual void use();
};

#endif