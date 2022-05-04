#ifndef SINGLE_TEXTURE_QUAD_SHADER_HPP
#define SINGLE_TEXTURE_QUAD_SHADER_HPP

#include "../PEShader.hpp"

class SingleTextureQuadShader : public PEShader {
public:
    static SingleTextureQuadShader* instance;

public:
    SingleTextureQuadShader();
    ~SingleTextureQuadShader();
};

#endif