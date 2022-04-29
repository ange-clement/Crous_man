#ifndef COMPONENT_SYSTEM_HPP
#define COMPONENT_SYSTEM_HPP

class Bitmap;

class ComponentSystem {
public:
    std::vector<unsigned short> entityIDs;

    Bitmap* requiredComponentsBitmap;
public:
    ComponentSystem();

    ~ComponentSystem();

    virtual void updateAll();
    virtual void updatePhysicsAll();
    virtual void updateAfterPhysicsAll();
    virtual void renderAll();
    virtual void updateAfterRenderAll();
    
    void initializeAll();

    void addEntity(unsigned short entityID);
    void removeEntity(unsigned short entityID);
    bool containsEntity(unsigned short entityID);

    unsigned short getComponentId(unsigned short entityID);

    virtual void update(unsigned short i, unsigned short entityID);
    virtual void updatePhysics(unsigned short i, unsigned short entityID);
    virtual void updateAfterPhysics(unsigned short i, unsigned short entityID);
    virtual void render(unsigned short i, unsigned short entityID);
    virtual void updateAfterRender(unsigned short i, unsigned short entityID);

    virtual void initialize(unsigned short i, unsigned short entityID) = 0;
    virtual void addEntityComponent() = 0;
};

#endif