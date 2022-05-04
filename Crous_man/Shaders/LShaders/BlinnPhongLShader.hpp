#ifndef BLINN_PHONG_SHADER_HPP
#define BLINN_PHONG_SHADER_HPP

#include "../LShader.hpp"

class BlinnPhongLShader : public LShader {
public:
    static BlinnPhongLShader* instance;

public:
    BlinnPhongLShader();
    ~BlinnPhongLShader();
};

#endif