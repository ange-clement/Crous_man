#ifndef BLINN_PHONG_SHADOW_SSAO_SHADER_HPP
#define BLINN_PHONG_SHADOW_SSAO_SHADER_HPP

#include "../LShader.hpp"

class BlinnPhongShadowSSAOLShader : public LShader {
public:
    static BlinnPhongShadowSSAOLShader* instance;

    GLuint gShadow;
    GLuint gSSAO;
    GLuint ambiantLocation;

public:
    BlinnPhongShadowSSAOLShader();
    ~BlinnPhongShadowSSAOLShader();

    void setAmbiant(float ambiant);

    virtual void useBuffers(const GLuint* buffers);
};

#endif