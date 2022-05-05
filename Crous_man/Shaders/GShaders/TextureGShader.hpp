#ifndef TEXTURE_G_SHADER_HPP
#define TEXTURE_G_SHADER_HPP

#include "../GShader.hpp"

struct Renderer;

class TextureGShader : public GShader {
public:
    static TextureGShader* instance;

    GLuint uDiffuseTexture;
    GLuint uSpecularTexture;

public:
    TextureGShader();
    ~TextureGShader();

    void setBuffers(GLuint uDiffuseTexture, GLuint uSpecularTexture);
    void prerender(const Renderer* r);
};

#endif