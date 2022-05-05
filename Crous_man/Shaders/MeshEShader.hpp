#ifndef MESH_E_SHADER_HPP
#define MESH_E_SHADER_HPP

#include <functional>

#include "MeshShader.hpp"


struct Renderer;

class FrameBuffer;

class MeshEShader : public MeshShader {
public:
    FrameBuffer* fBuffer;
    std::function<bool(Renderer*)> acceptRendererIf;
public:
    MeshEShader(std::string fs,
        unsigned int width, unsigned int height, unsigned int numberOfBuffers,
        std::function<bool(Renderer*)> acceptRendererIf);
    ~MeshEShader();

    void updateBufferWidthHeight(unsigned int width, unsigned int height);

    virtual void setOutputShaders(GLuint* buffers);

    virtual void use();
};

#endif