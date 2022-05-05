#ifndef COLLIDER_COMPONENT_HPP
#define COLLIDER_COMPONENT_HPP

#include <map>
#include <vector>
#include "../ECS/ComponentSystem.hpp"
#include "../Shaders/ColliderShader.hpp"


enum colliderType {
    Sphere,
    AABB,
    OBB,
};
struct Collider{
    unsigned short entityID;
    colliderType type;

    glm::vec3 center;
    glm::vec3 position;
    
    //For sphere data
    float radius;

    //For boxes data
    glm::vec3 size;
    //For OBB
    glm::mat3 orientation;

    //For rendering, empty list instead
    bool drawable = false;
    bool draw = false;

    std::vector<glm::vec3> vertices;
    std::vector<unsigned short> indices;
    GLuint vertexbuffer;
    GLuint elementbuffer;
    GLuint VertexArrayID;
    ColliderShader* shader;
};

class ColliderResult {
public :
    ColliderResult();
    ~ColliderResult();

    bool isInCollision;
    float penetrationDistance;
    glm::vec3 pointCollision;
};

// Map : clef = entityID
typedef std::map<unsigned short, std::vector<ColliderResult*>> CollisionResultMap;

class ColliderSystem : public ComponentSystem {
private :

    CollisionResultMap collisionResultMap;

    void simpleCollisionResolution();
    void QuadTreeCollisionResolution();
    void OcTreeCollisionResolution();

public :
    bool drawColliders;

    ColliderSystem();
    ~ColliderSystem();

    virtual void update(unsigned short i, unsigned short entityID);
    
    virtual void updateCollision(unsigned short i, unsigned short entityID);
    virtual void updateOnCollide(unsigned short i, unsigned short entityID);

    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    ColliderResult* intersect(Collider c1, Collider c2);
    virtual void updateCollisionAll();
    virtual void updateOnCollideAll();

    Collider* getCollider(unsigned short i);

    void renderAll(glm::mat4 view, glm::mat4 projection);
    void drawCollider(unsigned short i);
    bool isInContactWithSomething(unsigned short i);

    std::vector<ColliderResult*> getResultOf(unsigned int entityID);
};


void AABBfromMinMax(Collider& aabb, const glm::vec3 min, const glm::vec3 max);
void computeDynamicMinMaxAABB(const Collider& aabb, glm::vec3& min, glm::vec3& max);
void computeMinMaxAABB(const Collider& aabb, glm::vec3& min, glm::vec3& max);

//Find closest point to special data structure
glm::vec3 closestPointSphere(const Collider& sphere, const glm::vec3& point);
glm::vec3 closestPointAABB(const Collider& aabb, const glm::vec3& point);
glm::vec3 closestPointDynamicAABB(const Collider& aabb, const glm::vec3& point);
glm::vec3 closestPointOBB(const Collider& obb, const glm::vec3& point);
glm::vec3 closestPointPlane(const glm::vec3& point, const glm::vec3 normal_plan, float distance_to_origin);
glm::vec3 closestPointLine(const glm::vec3& point, const glm::vec3& start_line, const glm::vec3& end_line);

//Say if a point is in a special data structure
bool pointInSphere(const glm::vec3& p, const Collider& sphere);
bool pointInAABB(const glm::vec3& point, const Collider& aabb);
bool pointInDynamicAABB(const glm::vec3& point, const Collider& aabb);
bool pointInOBB(const glm::vec3& point, const Collider& obb);
bool pointOnPlane(const glm::vec3& point, const glm::vec3 normal_plan, float distance_to_origin);
bool pointOnLine(const glm::vec3& point, const glm::vec3& start_line, const glm::vec3& end_line);

//Collision resolution
#endif