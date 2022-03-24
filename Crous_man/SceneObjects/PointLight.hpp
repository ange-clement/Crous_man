#ifndef POINT_LIGHT_HPP
#define POINT_LIGHT_HPP

#include "../Scene.hpp"

#define MAX_LIGHTS 16

class PointLight : public virtual SceneObject {
public:

    PointLight();
};

#endif