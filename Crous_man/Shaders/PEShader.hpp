#ifndef PE_SHADER_HPP
#define PE_SHADER_HPP

#include "QuadShader.hpp"

class PEShader : public QuadShader {
public:
    GLuint colorTexture;

public:
    PEShader(std::string fs);
    ~PEShader();

    void use();
    void setColorTexture(GLuint colorTexture);
};

#endif