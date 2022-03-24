#ifndef TERRAIN_HPP
#define TERRAIN_HPP

#include "../Scene.hpp"


#include "../../external/image_ppm.h"

class Terrain : public SceneObject {
public:
    int nbPointsWidth = 128;
    float heightScale = 1.0;
    float heightTextureScale = 1.0;
    float textureScale = 10.0;

    Texture* heightTexture;
    Texture* grassTexture;
    Texture* rockTexture;
    Texture* snowrockTexture;

    GLuint heightScaleId;
    GLuint heightTextureScaleId;
    GLuint textureScaleId;

    OCTET* ImgHeight;
    int width;
    int height;

public:

    Terrain();
    Terrain(glm::vec3 start, float size);

    void updatePlaneMesh(glm::vec3 start, float size);
    float getHeight(glm::vec3 point);
    float lerp(float a, float b, float v);

    void draw(glm::mat4 view, glm::mat4 projection);
};

#endif