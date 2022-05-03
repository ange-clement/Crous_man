#ifndef COLLIDER_SHADER_HPP
#define COLLIDER_SHADER_HPP

#include "Shader.hpp"

class Collider;

class ColliderShader : public Shader {
public:
    static ColliderShader* instance;

    GLuint colliderColor;

    GLuint model;
    GLuint view;
    GLuint projection;
    GLuint normal;

    glm::vec3 onColidColor      = glm::vec3(1, 0, 0);
    glm::vec3 notOnColidColor   = glm::vec3(0, 1, 0);

public:
    ColliderShader();
    ~ColliderShader();
    void setMVPC(const glm::mat4& model, const glm::mat4& view, const glm::mat4& projection, bool incontact);

    void init(Collider* c);
    void draw(Collider* c);
};

#endif