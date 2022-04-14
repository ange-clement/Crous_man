#ifndef POINT_LIGHT_COMPONENT_HPP
#define POINT_LIGHT_COMPONENT_HPP

#include "../ECS/ComponentSystem.hpp"

struct PointLight {
    glm::vec3 color;
    
    float linear;
    float quadratic;

    PointLight() {
        color = glm::vec3(1.0, 1.0, 1.0);
        linear = .1f;
        quadratic = 0.02f;
    }
};

class PointLightSystem : public virtual ComponentSystem {
public:
    PointLightSystem();

    ~PointLightSystem();

    virtual void update(unsigned short i, unsigned short entityID);
    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    PointLight* getPointLight(unsigned short i);
};

#endif