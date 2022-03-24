#ifndef LODOBJECT_HPP
#define LODOBJECT_HPP

#include "../Scene.hpp"

class LODObject : public SceneObject {
public:
    unsigned int numberOfLevels;
    std::vector<Mesh*> meshes;

public:

    LODObject(std::vector<std::string> meshnames, std::vector<bool> hasNormal);

    void setLevel(unsigned int l);
};

#endif