#ifndef COMPONENT_SYSTEM_HPP
#define COMPONENT_SYSTEM_HPP

class Entity;
class Bitmap;

class ComponentSystem {
public:
    std::vector<unsigned short> entityIDs;

    Bitmap* requiredComponentsBitmap;
public:
    ComponentSystem();

    ~ComponentSystem();

    virtual void updateAll();
    void initializeAll();

    void addEntity(unsigned short entityID);

    void removeEntity(unsigned short entityID);

    unsigned short getComponentId(unsigned short entityID);
    Entity* getEntity(unsigned short entityID);


    virtual void update(unsigned short i, unsigned short entityID) = 0;
    virtual void initialize(unsigned short i, unsigned short entityID) = 0;
    virtual void addEntityComponent() = 0;
};

#endif