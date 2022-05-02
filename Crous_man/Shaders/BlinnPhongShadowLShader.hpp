#ifndef BLINN_PHONG_SHADOW_SHADER_HPP
#define BLINN_PHONG_SHADOW_SHADER_HPP

#include "LShader.hpp"

class BlinnPhongShadowLShader : public LShader {
public:
    static BlinnPhongShadowLShader* instance;

public:
    BlinnPhongShadowLShader();
    ~BlinnPhongShadowLShader();
};

#endif