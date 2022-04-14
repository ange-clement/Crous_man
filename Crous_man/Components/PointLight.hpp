#ifndef POINT_LIGHT_COMPONENT_HPP
#define POINT_LIGHT_COMPONENT_HPP

#include "../ECS/ComponentSystem.hpp"

struct PointLight {
    glm::vec3 color;
    
    float linear;
    float quadratic;
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