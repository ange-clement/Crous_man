#ifndef PLANET_HPP
#define PLANET_HPP

#include "../../Scene.hpp"

class Texture;

class Planet : public virtual SceneObject {
public:
    Texture* texture;
    GLuint UVBuffer;

    std::vector<glm::vec2> UV;

public:
    Planet(std::string texture, std::string vertexShader, std::string fragmentShader);

    void computeUV();

    void draw(glm::mat4 view, glm::mat4 projection);
};

#endif