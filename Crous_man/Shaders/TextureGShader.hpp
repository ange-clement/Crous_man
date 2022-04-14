#ifndef TEXTURE_G_SHADER_HPP
#define TEXTURE_G_SHADER_HPP

#include "GShader.hpp"

class TextureGShader : public GShader {
public:
    static TextureGShader* instance;

public:
    TextureGShader();
    ~TextureGShader();
};

#endif