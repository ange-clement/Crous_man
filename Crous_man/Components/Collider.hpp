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
    glm::vec3 dimensions;
    glm::vec3 size;
    //For OBB
    glm::mat3 orientation;

    //RENDERING DATA
    bool drawable = false;
    bool draw = false;
    ColliderShader* shader;
};

struct ContactPoint {
    ContactPoint();
    ~ContactPoint();

    void print();

    //Half of the total penetration length
    float penetrationDistance;
    glm::vec3 point;
    //Movement direction to avoid collision
    glm::vec3 normal;
};

class ColliderResult {
public :
    ColliderResult();
    ColliderResult(unsigned short id, ColliderResult *c);
    ~ColliderResult();

    void print();
    void inverseNormals();

    unsigned short entityCollidID;
    

    bool isInCollision;

    std::vector<ContactPoint*> contactsPts;
};

// Map : clef = entityID
typedef std::map<unsigned short, std::vector<ColliderResult*>> CollisionResultMap;
typedef std::map<unsigned short, std::vector<bool>> SimpleCollisionResultMap;

class ColliderSystem : public ComponentSystem {
private :
    CollisionResultMap collisionResultMap;
    SimpleCollisionResultMap simpleCollisionResultMap;

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

    std::vector<ColliderResult*> getResultOf(unsigned int entityID);


    //Intersection main function
    void computeIntersection(unsigned short i, unsigned short entityIID, unsigned short j, unsigned short entityJID);

    friend class RigidBody;
};

//To check if the collider provided is a computed collider
bool isSupportedCollider(Collider c);


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
void AABBAABBCollision(const Collider& aabb1, const Collider& aabb2, ColliderResult* res);
void SphereSphereCollision(Collider sp1, Collider sp2, ColliderResult* res);
void SphereAABBCollision(const Collider& aabb, const Collider& sphere, ColliderResult* res);
void SphereOBBCollision(const Collider& sphere, const Collider& obb, ColliderResult* res);
void SpherePlaneCollision(const Collider& sphere, const glm::vec3 normal_plan, float distance_to_origin, ColliderResult* res);
void AABBOBBCollision(const Collider& aabb, const Collider& obb, ColliderResult* res);
void AABBPlaneCollision(const Collider& aabb, const glm::vec3 normal_plan, float distance_to_origin, ColliderResult* res);
void OBBOBBCollision(const Collider& obb1, const Collider& obb2, ColliderResult* res);
void OBBPlaneCollision(const Collider& obb, const glm::vec3 normal_plan, float distance_to_origin, ColliderResult* res);

//For OBB Special treatment
void getVerticesOBB(const Collider& obb, std::vector<glm::vec3>& vertex);
void getEdgesOBB(const Collider& obb, std::vector<glm::vec3>& start_lines, std::vector<glm::vec3>& end_lines);
void getPlanesOBB(const Collider& obb, std::vector<glm::vec3>& normals_plane, std::vector<float>& distances_to_origin);
bool clipToPlane(const glm::vec3& normal_plane,const float distance_to_origin_plane, const glm::vec3& start_line, const glm::vec3& end_line, glm::vec3* outPoint);
std::vector<glm::vec3> clipEdgesToOBB(const std::vector<glm::vec3>& start_edges, const std::vector<glm::vec3>& end_edges, const Collider& obb);
float penetrationDepthOBB(const Collider& o1, const Collider& o2, const glm::vec3& axis, bool* outShouldFlip);

//For AABB Special treatment
void getVerticesAABB(const Collider& aabb, std::vector<glm::vec3>& vertex);
void getEdgesAABB(const Collider& aabb, std::vector<glm::vec3>& start_lines, std::vector<glm::vec3>& end_lines);
void getPlanesAABB(const Collider& aabb, std::vector<glm::vec3>& normals_plane, std::vector<float>& distances_to_origin);
std::vector<glm::vec3> clipEdgesToAABB(const std::vector<glm::vec3>& start_edges, const std::vector<glm::vec3>& end_edges, const Collider& aabb);
float penetrationDepthAABBOBB(const Collider& aabb, const Collider& obb, const glm::vec3& axis, bool* outShouldFlip);
//Have collision result and resolution data
void intersect(Collider c1, Collider c2, ColliderResult* res);

//Just to see if there is an intersection
bool isInIntersection(Collider c1, Collider c2);

bool isAABBAABBCollision(const Collider& aabb1, const Collider& aabb2);
bool isSphereSphereCollision(Collider sp1, Collider sp2);
bool isSphereAABBCollision(const Collider& aabb, const Collider& sphere);
bool isSphereOBBCollision(const Collider& sphere, const Collider& obb);
bool isSpherePlaneCollision(const Collider& sphere, const glm::vec3 normal_plan, float distance_to_origin);
bool isAABBOBBCollision(const Collider& aabb, const Collider& obb);
bool isAABBPlaneCollision(const Collider& aabb, const glm::vec3 normal_plan, float distance_to_origin);
bool isOBBOBBCollision(const Collider& obb1, const Collider& obb2);
bool isOBBPlaneCollision(const Collider& obb, const glm::vec3 normal_plan, float distance_to_origin);

bool PlanePlaneIntersection(const glm::vec3 normal_plan_1, float distance_to_origin_1, const glm::vec3 normal_plan_2, float distance_to_origin_2);
#endif