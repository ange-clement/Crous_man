#ifndef DELETE_AFTER_TIME_COMPONENT_HPP
#define DELETE_AFTER_TIME_COMPONENT_HPP

#include "../ECS/ComponentSystem.hpp"

struct DeleteAfterTime {
    float timeLeftToDelete = 4.0f;
    float timeBeforeScaling = 3.0f;
    float invTimeDiff = -1.0f;
    glm::vec3 originalScale = glm::vec3(-1.0f);
};

class DeleteAfterTimeSystem : public virtual ComponentSystem {
public:
    DeleteAfterTimeSystem();

    ~DeleteAfterTimeSystem();

    virtual void update(unsigned short i, unsigned short entityID);
    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    DeleteAfterTime* getDeleteAfterTime(unsigned short i);
};

#endif