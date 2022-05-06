#ifndef COLLIDER_COMPONENT_HPP
#define COLLIDER_COMPONENT_HPP

#include <map>
#include <vector>
#include "../ECS/ComponentSystem.hpp"
#include "../Shaders/ColliderShader.hpp"

class RaycastResult;
class Ray;

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
    glm::vec3 dimensions;
    glm::vec3 size;
    //For OBB
    glm::mat3 orientation;

    //RENDERING DATA
    bool drawable = false;
    bool draw = false;
    ColliderShader* shader;
};

class ColliderResult {
public :
    ColliderResult();
    ColliderResult(unsigned short id, ColliderResult *c);
    ~ColliderResult();

    unsigned short entityCollidID;
    bool isInCollision;
    float penetrationDistance;
    glm::vec3 pointCollision;
};

// Map : clef = entityID
typedef std::map<unsigned short, std::vector<ColliderResult*>> CollisionResultMap;

class ColliderSystem : public ComponentSystem {
private :

    CollisionResultMap collisionResultMap;
    std::vector<glm::vec3> verticesCube;

    //Collision treatment
    void simpleCollisionResolution();
    void QuadTreeCollisionResolution();
    void OcTreeCollisionResolution();

    void clearAllCollision(unsigned short i);
    
    //AABB
    void computeAABBDimensions(Collider* c);

public :

    ColliderSystem();
    ~ColliderSystem();

    virtual void update(unsigned short i, unsigned short entityID);
    
    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    
    
    virtual void updateCollision(unsigned short i, unsigned short entityID);
    virtual void updateOnCollide(unsigned short i, unsigned short entityID);
    virtual void updateCollisionAll();
    virtual void updateOnCollideAll();

    Collider* getCollider(unsigned short i);
    Collider* getColliderEntityID(unsigned short entityID);

    void renderAll(glm::mat4 view, glm::mat4 projection);
    void drawCollider(unsigned short i);
    bool isInContactWithSomething(unsigned short i);

    std::vector<RaycastResult*> rayCastAll(const Ray& ray);

    std::vector<ColliderResult*> getResultOf(unsigned int entityID);
};


//Find closest point to special data structure
glm::vec3 closestPointSphere(const Collider& sphere, const glm::vec3& point);
glm::vec3 closestPointAABB(const Collider& aabb, const glm::vec3& point);
glm::vec3 closestPointOBB(const Collider& obb, const glm::vec3& point);
glm::vec3 closestPointPlane(const glm::vec3& point, const glm::vec3 normal_plan, float distance_to_origin);

//Say if a point is in a special data structure
bool pointInSphere(const glm::vec3& p, const Collider& sphere);
bool pointInAABB(const glm::vec3& point, const Collider& aabb);
bool pointInOBB(const glm::vec3& point, const Collider& obb);
bool pointOnPlane(const glm::vec3& point, const glm::vec3 normal_plan, float distance_to_origin);


void logCollisionResultMap(CollisionResultMap m);

void AABBfromMinMax(Collider& aabb, const glm::vec3 min, const glm::vec3 max);
void computeMinMaxAABB(const Collider& c, glm::vec3& min, glm::vec3& max);


//INtersection between collider structure data
ColliderResult* AABBAABBCollision(const Collider& aabb1, const Collider& aabb2);
ColliderResult* SphereSphereCollision(Collider sp1, Collider sp2);
ColliderResult* SphereAABBCollision(const Collider& aabb, const Collider& sphere);
ColliderResult* SphereAABBCollision_bis(const Collider& aabb, const Collider& sphere);
ColliderResult* SphereOBBCollision(const Collider& sphere, const Collider& obb);
ColliderResult* SpherePlaneCollision(const Collider& sphere, const glm::vec3 normal_plan, float distance_to_origin);
ColliderResult* AABBOBBCollision(const Collider& aabb, const Collider& obb);
ColliderResult* AABBPlaneCollision(const Collider& aabb, const glm::vec3 normal_plan, float distance_to_origin);
ColliderResult* OBBOBBCollision(const Collider& obb1, const Collider& obb2);
ColliderResult* OBBPlaneCollision(const Collider& obb, const glm::vec3 normal_plan, float distance_to_origin);

ColliderResult* intersect(Collider c1, Collider c2);

bool PlanePlaneIntersection(const glm::vec3 normal_plan_1, float distance_to_origin_1, const glm::vec3 normal_plan_2, float distance_to_origin_2);
#endif