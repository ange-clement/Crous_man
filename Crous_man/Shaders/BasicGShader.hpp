#ifndef BASIC_G_SHADER_HPP
#define BASIC_G_SHADER_HPP

#include "GShader.hpp"

class BasicGShader : public GShader {
public:
    static BasicGShader* instance;

public:
    BasicGShader();
    ~BasicGShader();
};

#endif