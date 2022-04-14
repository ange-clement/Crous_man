#ifndef G_SHADER_HPP
#define G_SHADER_HPP

#include "MeshShader.hpp"

struct Renderer;

class GShader : public MeshShader {
public:

public:
    GShader(std::string fs);
    ~GShader();

    virtual void prerender(const Renderer* r);
};

#endif