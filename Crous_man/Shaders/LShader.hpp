#ifndef L_SHADER_HPP
#define L_SHADER_HPP

#include "QuadShader.hpp"

class LShader : public QuadShader {
public:
    GLuint gPosition;
    GLuint gNormal;
    GLuint gAlbedo;

    GLuint viewPos;

    //GLuint light;

public:
    LShader(std::string fs);
    ~LShader();

    void use();
    void setViewPos(glm::vec3 viewPos);
    void setBuffers(GLuint gPosition, GLuint gNormal, GLuint gAlbedo);
    virtual void useBuffers(const GLuint* buffers);
    void setLight(unsigned int index, glm::vec3 position, glm::vec3 color, float linear, float quadratic);
};

#endif