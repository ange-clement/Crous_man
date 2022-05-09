#ifndef ENTITY_HPP
#define ENTITY_HPP

#include "SystemIDs.hpp"

class Bitmap;
class Transform;

class Entity {
public:
    unsigned short id;
    Bitmap* componentsBitmap;

    bool isActive;

    Transform* transform;
    Transform* worldTransform;

    //Hierarchy
    Entity* parent;
    std::vector<Entity*> childrens;

public:
    Entity();

    Entity(std::initializer_list<SystemIDs> systems);

    ~Entity();

    void addChildren(Entity* children);
    void removeAllChildrens();

    void addComponent(SystemIDs componentId);
    bool removeComponent(SystemIDs componentId);

    void setActiveRecursive(bool activeStatus);

    void updateTransforms();
};

#endif