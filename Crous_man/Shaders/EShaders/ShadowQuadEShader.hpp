#ifndef SHADOW_QUAD_E_SHADER_HPP
#define SHADOW_QUAD_E_SHADER_HPP

#include "../QuadEShader.hpp"

class ShadowQuadEShader : public QuadEShader {
public:
    static ShadowQuadEShader* instance;

    GLuint uShadowMapLocation;
    GLuint gPositionLocation;
    GLuint uLightSpaceMatrixLocation;

public:
    ShadowQuadEShader();
    ~ShadowQuadEShader();

    void setShadowMap(GLuint shadowMap);
    void setPosition(GLuint gPosition);
    void setLightSpaceMatrix(glm::mat4 lightSpaceMatrix);

    virtual void useBuffers(std::vector<GLuint> buffers);
};

#endif