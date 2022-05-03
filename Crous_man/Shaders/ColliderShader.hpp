#ifndef COLLIDER_SHADER_HPP
#define COLLIDER_SHADER_HPP

#include "Shader.hpp"

class ColliderShader : public Shader {
public:
    static ColliderShader* instance;
    GLuint colliderColor;

    GLuint model;
    GLuint view;
    GLuint projection;
    GLuint normal;

public:
    ColliderShader();
    ~ColliderShader();
    void setMVP(const glm::mat4& model, const glm::mat4& view, const glm::mat4& projection);
};
#endif