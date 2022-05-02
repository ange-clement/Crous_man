#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "../InputManager.hpp"
#include "../ECS/EntityManager.hpp"
#include "../ECS/Bitmap.hpp"
#include "../ECS/Entity.hpp"
#include "../Transform.hpp"
#include "../Util.hpp"

#include "Collider.hpp"

void logCollisionResultMap(CollisionResultMap m) {
    for (auto const& x : m){
        std::cout << x.first  << " : [" << std::endl;
        for (auto const& el : x.second) {
            std::cout << el.isInCollision << "," << el.penetrationDistance << "{" << el.pointCollision.x << "," << el.pointCollision.y << "," << el.pointCollision.z << "}" << std::endl;
        }
        std::cout << "]" << std::endl;
    }
}

void ColliderSystem::initCollisionResultMap() {
    unsigned short entityID;
    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        entityID = entityIDs[i];
        std::vector<ColliderResult*> res;
        res.reserve(entityIDs.size()-1);
        collisionResultMap.insert(CollisionResultMap::value_type(entityID,res));
    }
}

ColliderSystem::ColliderSystem() : ComponentSystem() {
    requiredComponentsBitmap = new Bitmap({SystemIDs::ColliderID});
    initCollisionResultMap();
}

ColliderSystem::~ColliderSystem() {}

void ColliderSystem::initialize(unsigned short i, unsigned short entityID) {
    getCollider(i)->entityID    = entityID;
    getCollider(i)->position    = glm::vec3(0);
    getCollider(i)->radius      = 1.0f;
    getCollider(i)->type        = colliderType::Sphere;
    getCollider(i)->size        = glm::vec3(0);
    getCollider(i)->orientation = EntityManager::instance->entities[entityID]->worldTransform->rotation.rotationMatrix;
}

void ColliderSystem::update(unsigned short i, unsigned short entityID) {
    //We move all the positions from all colliders
    Entity* entity = EntityManager::instance->entities[entityID];
    entity->worldTransform->applyToPoint(getCollider(i)->position);
}

void ColliderSystem::updatePhysics(unsigned short i, unsigned short entityID) {
    //Update collisions

}
void ColliderSystem::addEntityComponent() {
    EntityManager::instance->colliderComponents.push_back(Collider());
}


//General intersection function
void ColliderSystem::computeAllIntersections() {
    //Iterate over all colliders and compute collision answers
    simpleCollisionResolution();
    logCollisionResultMap(this->collisionResultMap);
}

void ColliderSystem::simpleCollisionResolution(unsigned short i, unsigned short entityID) {
    unsigned short entityIDJ;
    Collider* c_i = getCollider(i);

    for (size_t j = 0, size = entityIDs.size(); j < size; j++) {
        if (!collisionResultMap[i][j]) {
            entityIDJ = entityIDs[j];
            Collider* c_j = getCollider(j);

        }
        
    }
}


void ColliderSystem::simpleCollisionResolution() {
    unsigned short entityID_first;
    unsigned short entityID_second;

    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        entityID_first = entityIDs[i];
        Collider* c_first = getCollider(i);
        for (size_t j = i+1; j < entityIDs.size(); j++){
            entityID_second = entityIDs[j];
            Collider* c_second = getCollider(j);
            ColliderResult* res;
            intersect(*c_first,*c_second,res);
            
            collisionResultMap[i][j] = res;
            collisionResultMap[j][i] = res;
        }
    }
}


void ColliderSystem::QuadTreeCollisionResolution() {

}
void ColliderSystem::OcTreeCollisionResolution() {

}

Collider* ColliderSystem::getCollider(unsigned short i) {
    return &EntityManager::instance->colliderComponents[i];
}


/* =================== =================== COLLISION DETECTION FUNCTIONS =================== ===================*/
//create AABB form a min and a max position only
void AABBfromMinMax(Collider& aabb, const glm::vec3 min, const glm::vec3 max) {
    aabb.type = colliderType::AABB;
    aabb.position = (min + max) * 0.5f;
    aabb.size = (max - min) * 0.5f;
}

//compute AABB min and max with dynamic AABB
void computeDynamicMinMaxAABB(const Collider& aabb, glm::vec3& min, glm::vec3& max) {
    assert(aabb.type == colliderType::AABB);

    //We need to take the posible rotation in care her
    for (size_t i = 0; i < 3; i++) {
        glm::vec3 aabb_Axisi = aabb.orientation[i] * aabb.size[i];
        minVec3(min, aabb_Axisi, min);
        maxVec3(max, aabb_Axisi, max);
        minVec3(min, glm::vec3(0) - aabb_Axisi, min);
        maxVec3(max, glm::vec3(0) - aabb_Axisi, max);
    }
}

//compute AABB min and max with static AABB
void computeMinMaxAABB(const Collider& aabb, glm::vec3& min, glm::vec3& max) {
    assert(aabb.type == colliderType::AABB);

    glm::vec3 p1 = aabb.position + aabb.size;
    glm::vec3 p2 = aabb.position - aabb.size;

    minVec3(p1, p2, min);
    maxVec3(p1, p2, max);
}


//Point in collider detection
bool pointInSphere(const glm::vec3& p, const Collider& sphere) {
    assert(sphere.type == colliderType::Sphere);
    return glm::length(p - sphere.position) < sphere.radius * sphere.radius;
}

bool pointInAABB(const glm::vec3& point, const Collider& aabb) {
    assert(aabb.type == colliderType::AABB);

    glm::vec3 min;
    glm::vec3 max;
    computeMinMaxAABB(aabb, min, max);

    if (point.x < min.x || point.y < min.y || point.z < min.z) {
        return false;
    }
    if (point.x > max.x || point.y > max.y || point.z > max.z) {
        return false;
    }
    return true;
}

bool pointInDynamicAABB(const glm::vec3& point, const Collider& aabb) {
    assert(aabb.type == colliderType::AABB);

    glm::vec3 min;
    glm::vec3 max;
    computeDynamicMinMaxAABB(aabb, min, max);

    if (point.x < min.x || point.y < min.y || point.z < min.z) {
        return false;
    }
    if (point.x > max.x || point.y > max.y || point.z > max.z) {
        return false;
    }
    return true;
}

bool pointInOBB(const glm::vec3& point, const Collider& obb) {
    assert(obb.type == colliderType::OBB);

    glm::vec3 dir = point - obb.position;

    for (int i = 0; i < 3; ++i) {
        glm::vec3 axis = obb.orientation[i];

        float distance = glm::dot(dir, axis);
        if (distance > obb.size[i]) {
            return false;
        }
        if (distance < -obb.size[i]) {
            return false;
        }
    }
    return true;
}

bool pointOnPlane(const glm::vec3& point, const glm::vec3 normal_plan, float distance_to_origin) {
    return compareWithEpsilon(glm::dot(point, normal_plan) - distance_to_origin, 0.0f);
}

bool pointOnLine(const glm::vec3& point, const glm::vec3& start_line, const glm::vec3& end_line) {
    glm::vec3 closest = closestPointLine(point, start_line, end_line);
    float distanceSq = glm::length(closest - point);
    return compareWithEpsilon(distanceSq, 0.0f);
}


//Find closest point between collider and another collider position
glm::vec3 closestPointSphere(const Collider& sphere, const glm::vec3& point) {
    assert(sphere.type == colliderType::Sphere);

    glm::vec3 sphereToPoint = glm::normalize(point - sphere.position);
    sphereToPoint = sphereToPoint * sphere.radius;
    return sphereToPoint + sphere.position;
}

glm::vec3 closestPointAABB(const Collider& aabb, const glm::vec3& point) {
    assert(aabb.type == colliderType::AABB);

    glm::vec3 result = point;
    glm::vec3 min;
    glm::vec3 max;
    computeMinMaxAABB(aabb, min, max);

    //Clamp to min
    result.x = (result.x < min.x) ? min.x : result.x;
    result.y = (result.y < min.x) ? min.y : result.y;
    result.z = (result.z < min.x) ? min.z : result.z;

    //Clamp to max
    result.x = (result.x > max.x) ? max.x : result.x;
    result.y = (result.y > max.x) ? max.y : result.y;
    result.z = (result.z > max.x) ? max.z : result.z;

    return result;
}

glm::vec3 closestPointDynamicAABB(const Collider& aabb, const glm::vec3& point) {
    assert(aabb.type == colliderType::AABB);

    glm::vec3 result = point;
    glm::vec3 min;
    glm::vec3 max;
    computeDynamicMinMaxAABB(aabb, min, max);

    //Clamp to min
    result.x = (result.x < min.x) ? min.x : result.x;
    result.y = (result.y < min.x) ? min.y : result.y;
    result.z = (result.z < min.x) ? min.z : result.z;

    //Clamp to max
    result.x = (result.x > max.x) ? max.x : result.x;
    result.y = (result.y > max.x) ? max.y : result.y;
    result.z = (result.z > max.x) ? max.z : result.z;

    return result;
}

glm::vec3 closestPointOBB(const Collider& obb, const glm::vec3& point) {
    assert(obb.type == colliderType::OBB);
    
    glm::vec3 result = obb.position;
    glm::vec3 dir = point - obb.position;

    //Now we iterate on the orientation of OBB
    for (int i = 0; i < 3; ++i) {
        glm::vec3 axis = obb.orientation[i];

        float distance = glm::dot(dir, axis);

        if (distance > obb.size[i]) {
            distance = obb.size[i];
        }
        if (distance < -obb.size[i]) {
            distance = -obb.size[i];
        }

        result = result + (axis * distance);
    }

    return result;
}

glm::vec3 closestPointPlane(const glm::vec3& point, const glm::vec3 normal_plan, float distance_to_origin) {
    float distance = glm::dot(normal_plan, point) - distance_to_origin;
    return point - normal_plan * distance;
}

glm::vec3 closestPointLine(const glm::vec3& point, const glm::vec3& start_line, const glm::vec3& end_line) {
    glm::vec3 lVec = end_line - start_line;
    float t = glm::dot(point - start_line, lVec) / glm::dot(lVec, lVec);
    // Clamp t to the 0 to 1 range
    t = ((t < 0.0f) ? 0.0f : ((t > 1.0f) ? 1.0f : t));
    // Return projected position of t
    return start_line + lVec * t;
}


//Method resolving axis overlapping with interval structure
typedef struct Interval {
    float min;
    float max;
} Interval;

Interval GetIntervalOBB(const Collider& obb, const glm::vec3& axis) {
    assert(obb.type == colliderType::OBB);

    glm::vec3 vertex[8];

    glm::vec3 C = obb.position;	// OBB Center
    glm::vec3 E = obb.size;		// OBB Extents

    //obb.orientation[0] => col 0 (first axis of the rotation)
    vertex[0] = C + obb.orientation[0] * E[0] + obb.orientation[1] * E[1] + obb.orientation[2] * E[2];
    vertex[1] = C - obb.orientation[0] * E[0] + obb.orientation[1] * E[1] + obb.orientation[2] * E[2];
    vertex[2] = C + obb.orientation[0] * E[0] - obb.orientation[1] * E[1] + obb.orientation[2] * E[2];
    vertex[3] = C + obb.orientation[0] * E[0] + obb.orientation[1] * E[1] - obb.orientation[2] * E[2];
    vertex[4] = C - obb.orientation[0] * E[0] - obb.orientation[1] * E[1] - obb.orientation[2] * E[2];
    vertex[5] = C + obb.orientation[0] * E[0] - obb.orientation[1] * E[1] - obb.orientation[2] * E[2];
    vertex[6] = C - obb.orientation[0] * E[0] + obb.orientation[1] * E[1] - obb.orientation[2] * E[2];
    vertex[7] = C - obb.orientation[0] * E[0] - obb.orientation[1] * E[1] + obb.orientation[2] * E[2];

    Interval result;
    result.min = result.max = glm::dot(axis, vertex[0]);

    for (int i = 1; i < 8; ++i) {
        float projection = glm::dot(axis, vertex[i]);
        result.min = (projection < result.min) ? projection : result.min;
        result.max = (projection > result.max) ? projection : result.max;
    }
    return result;
}

Interval GetIntervalAABB(const Collider& aabb, const glm::vec3& axis, bool dynamicAABB = true) {
    assert(aabb.type == colliderType::AABB);
    
    glm::vec3 i;
    glm::vec3 a;
    if (dynamicAABB) {
        computeDynamicMinMaxAABB(aabb, i,a);
    }
    else {
        computeMinMaxAABB(aabb, i, a);
    }

    glm::vec3 vertex[8] = {
        glm::vec3(i.x, a.y, a.z),
        glm::vec3(i.x, a.y, i.z),
        glm::vec3(i.x, i.y, a.z),
        glm::vec3(i.x, i.y, i.z),
        glm::vec3(a.x, a.y, a.z),
        glm::vec3(a.x, a.y, i.z),
        glm::vec3(a.x, i.y, a.z),
        glm::vec3(a.x, i.y, i.z)
    };

    Interval result;
    result.min = result.max = glm::dot(axis, vertex[0]);

    for (int i = 1; i < 8; ++i) {
        float projection = glm::dot(axis, vertex[i]);
        result.min = (projection < result.min) ? projection : result.min;
        result.max = (projection > result.max) ? projection : result.max;
    }
    return result;
}

ColliderResult OverlapOnAxisAABBOBB(const Collider& aabb, const Collider& obb, const glm::vec3& axis, bool dynamicAABB = true) {
    assert(aabb.type == colliderType::AABB);
    assert(obb.type == colliderType::OBB);

    ColliderResult res;
    res.pointCollision = axis;

    Interval a = GetIntervalAABB(aabb, axis, dynamicAABB);
    Interval b = GetIntervalOBB(obb, axis);

    res.isInCollision = ((b.min <= a.max) && (a.min <= b.max));
    res.penetrationDistance = std::min(a.max-b.min,b.max-a.min);
    return res;
}

ColliderResult OverlapOnAxisOBBOBB(const Collider& obb1, const Collider& obb2, const glm::vec3& axis) {
    assert(obb1.type == colliderType::OBB);
    assert(obb2.type == colliderType::OBB);

    ColliderResult res;
    res.pointCollision = axis;
    Interval a = GetIntervalOBB(obb1, axis);
    Interval b = GetIntervalOBB(obb1, axis);

    res.isInCollision = ((b.min <= a.max) && (a.min <= b.max));
    res.penetrationDistance = std::min(a.max - b.min, b.max - a.min);
    return res;
}


//Functions for compute type of collisions
//Collision between two AABB
//TODO TEST
ColliderResult AABBAABBCollision(const Collider& aabb1, const Collider& aabb2, bool dynamicAABB = true) {
    assert(aabb1.type == colliderType::AABB);
    assert(aabb2.type == colliderType::AABB);

    ColliderResult res;
    res.pointCollision = glm::vec3(0);
    res.penetrationDistance = 0;
    res.isInCollision = false;

    glm::vec3 min_1;
    glm::vec3 min_2;
    glm::vec3 max_1;
    glm::vec3 max_2;
    if (dynamicAABB) {
        computeDynamicMinMaxAABB(aabb1, min_1, max_1);
        computeDynamicMinMaxAABB(aabb2, min_2, max_2);
    }
    else {
        computeMinMaxAABB(aabb1, min_1, max_1);
        computeMinMaxAABB(aabb2, min_2, max_2);
    }

    bool overlap_x = ((aabb1.position.x + min_1.x) < (aabb2.position.x + max_2.x)) && ((aabb2.position.x + min_2.x) < (aabb1.position.x + max_1.x));
    bool overlap_y = ((aabb1.position.y + min_1.y) < (aabb2.position.y + max_2.y)) && ((aabb2.position.y + min_2.y) < (aabb1.position.y + max_1.y));
    bool overlap_z = ((aabb1.position.z + min_1.z) < (aabb2.position.z + max_2.z)) && ((aabb2.position.z + min_2.z) < (aabb1.position.z + max_1.z));

    bool incontact = overlap_x && overlap_y && overlap_z;
    if (incontact) {
        res.isInCollision = true;
        //If the two colliders are in contact
        //try to find the min distance to split them

        float dist_x_1 = (aabb1.position.x + max_1.x) - (aabb2.position.x + min_2.x);
        float dist_x_2 = (aabb2.position.x + max_2.x) - (aabb1.position.x + min_1.x);
        float dist_y_1 = (aabb1.position.y + max_1.y) - (aabb2.position.y + min_2.y);
        float dist_y_2 = (aabb2.position.y + max_2.y) - (aabb1.position.y + min_1.y);
        float dist_z_1 = (aabb1.position.z + max_1.z) - (aabb2.position.z + min_2.z);
        float dist_z_2 = (aabb2.position.z + max_2.z) - (aabb1.position.z + min_1.z);


        float x_factor = (dist_x_1 < dist_x_2) ? 1 : -1;
        float y_factor = (dist_y_1 < dist_y_2) ? 1 : -1;
        float z_factor = (dist_z_1 < dist_z_2) ? 1 : -1;

        float x = (dist_x_1 < dist_x_2) ? dist_x_1 : dist_x_2;
        float y = (dist_y_1 < dist_y_2) ? dist_y_1 : dist_y_2;
        float z = (dist_z_1 < dist_z_2) ? dist_z_1 : dist_z_2;

        if (x <= z && x <= y) {
            res.penetrationDistance = x;
            z = 0;
            y = 0;
        }
        else if (y <= x && y <= z) {
            res.penetrationDistance = y;
            x = 0;
            z = 0;
        }
        else if (z <= y && z <= x) {
            res.penetrationDistance = z;
            x = 0;
            y = 0;
        }
        res.pointCollision = glm::normalize(glm::vec3(x * x_factor, y * y_factor, z * z_factor));
    }
    return res;
}


//Collision between two Sphere
//TODO TEST
ColliderResult SphereSphereCollision(Collider sp1, Collider sp2) {
    assert(sp1.type == colliderType::Sphere);
    assert(sp2.type == colliderType::Sphere);

    ColliderResult res;
    res.isInCollision = false;

    glm::vec3 mvt = sp1.position - sp2.position;
    float distance = glm::length(mvt);
    if (distance < (sp1.radius + sp2.radius)) {
        res.isInCollision = true;
        res.penetrationDistance = (sp1.radius - distance) + sp2.radius;
        res.pointCollision = ((distance == 0) ? glm::vec3(0, 1, 0) : glm::normalize(mvt));
    }
    return res;
}

glm::vec3 perform_direction(glm::vec3 target, float& penetration, float radius) {
    glm::vec3 compass[] = {
        glm::vec3(0.0f, 1.0f,0.0f),	    // UP
        glm::vec3(0.0f, -1.0f,0.0f),	// DOWN
        glm::vec3(1.0f, 0.0f,0.0f),	    // RIGHT
        glm::vec3(-1.0f, 0.0f,0.0f),	// LEFT
        glm::vec3(0.0f, 0.0f,-1.0f),	// BEHIND
        glm::vec3(0.0f, 0.0f, 1.0f)	    // FRONT
    };

    glm::vec3 n_target = glm::normalize(target);
    float max = 0.0f;
    unsigned int best_dir = -1;

    for (unsigned int i = 0; i < 6; i++) {
        float dot_product = glm::dot(n_target, compass[i]);
        if (dot_product > max) {
            max = dot_product;
            best_dir = i;
        }
    }

    switch (best_dir) {
    case 0:
    case 1:
        penetration = radius - std::abs(target.y);
        break;
    case 2:
    case 3:
        penetration = radius - std::abs(target.x);
        break;
    case 4:
    case 5:
        penetration = radius - std::abs(target.z);
        break;
    }

    return ((best_dir >= 0) ? compass[best_dir] : glm::vec3(0));
}
//Collision between an AABB and a sphere
//TODO TEST
ColliderResult SphereAABBCollision(const Collider& aabb, const Collider& sphere, bool dynamicAABB = true) {
    assert(aabb.type == colliderType::AABB);
    assert(sphere.type == colliderType::Sphere);
    
    ColliderResult res;
    res.pointCollision = glm::vec3(0);
    res.penetrationDistance = 0;
    res.isInCollision = false;

    glm::vec3 min;
    glm::vec3 max;

    if (dynamicAABB) {
        computeMinMaxAABB(aabb, min, max);
    }
    else {
        computeDynamicMinMaxAABB(aabb, min, max);
    }

    glm::vec3 difference = sphere.position - aabb.position;

    //Get the coord of the point in the sphere at smalest dist from the cube
    float x = std::max(min.x, std::min(difference.x, max.x));
    float y = std::max(min.y, std::min(difference.y, max.y));
    float z = std::max(min.z, std::min(difference.z, max.z));


    difference = (glm::vec3(x, y, z) + aabb.position) - sphere.position;
    float distance = glm::length(difference);

    if (distance < sphere.radius) {
        //We got a contact
        res.isInCollision = true;

        //If both are in the same place
        if (distance == 0) {
            //Get min dist to get out of other collider
            glm::vec3 dist = max - min;

            if (dist.y <= dist.x && dist.y <= dist.z) {
                res.penetrationDistance = dist.y + sphere.radius;
                res.pointCollision = glm::vec3(0, 1, 0);
            }
            else if (dist.x <= dist.y && dist.x <= dist.z) {
                res.penetrationDistance = dist.x + sphere.radius;
                res.pointCollision = glm::vec3(1, 0, 0);
            }
            else if (dist.z <= dist.y && dist.z <= dist.x) {
                res.penetrationDistance = dist.z + sphere.radius;
                res.pointCollision = glm::vec3(0, 0, 1);
            }
        }
        else {
            //We need to know the direction of the minimum penetration
            res.pointCollision = perform_direction(difference, res.penetrationDistance, sphere.radius);
        }
    }
    return res;
}

//TODO TEST
ColliderResult SphereAABBCollision_bis(const Collider& aabb, const Collider& sphere) {
    assert(aabb.type == colliderType::AABB);
    assert(sphere.type == colliderType::Sphere);

    ColliderResult res;
    res.pointCollision = glm::vec3(0);
    res.penetrationDistance = 0;
    res.isInCollision = false;

    glm::vec3 closestPoint = closestPointAABB(aabb, sphere.position);
    glm::vec3 dif = sphere.position - closestPoint;
    float dist_squared = glm::dot(dif,dif);
    float radius_squared = sphere.radius * sphere.radius;

    if (dist_squared < radius_squared) {
        //We got a collision then
        glm::vec3 closestPointSp    = closestPointSphere(sphere, aabb.position);
        dif = closestPoint - closestPointSp;
        res.pointCollision          = glm::normalize(dif);
        res.penetrationDistance     = glm::length(dif);
        res.isInCollision           = true;
    }
    return res;
}

//TODO TEST
ColliderResult SphereOBBCollision(const Collider& sphere, const Collider& obb) {
    assert(sphere.type == colliderType::Sphere);
    assert(obb.type == colliderType::OBB);

    ColliderResult res;
    res.pointCollision = glm::vec3(0);
    res.penetrationDistance = 0;
    res.isInCollision = false;

    glm::vec3 closestPoint = closestPointOBB(obb, sphere.position);
    glm::vec3 dif = sphere.position - closestPoint;
    float dist_squared = glm::dot(dif,dif);
    float radius_squared = sphere.radius * sphere.radius;

    if (dist_squared < radius_squared) {
        //We got a collision then
        glm::vec3 closestPointSp    = closestPointSphere(sphere, obb.position);
        dif = closestPoint - closestPointSp;
        res.pointCollision          = glm::normalize(dif);
        res.penetrationDistance     = glm::length(dif);
        res.isInCollision           = true;
    }
    return res;
}

//TODO TEST
ColliderResult SpherePlaneCollision(const Collider& sphere, const glm::vec3 normal_plan, float distance_to_origin) {
    assert(sphere.type == colliderType::Sphere);

    ColliderResult res;
    res.pointCollision = glm::vec3(0);
    res.penetrationDistance = 0;
    res.isInCollision = false;

    glm::vec3 closestPoint = closestPointPlane(sphere.position, normal_plan,distance_to_origin);
    glm::vec3 dif = sphere.position - closestPoint;
    float dist_squared = glm::dot(dif, dif);
    float radius_squared = sphere.radius * sphere.radius;

    if (dist_squared < radius_squared) {
        //We got a collision then       
        res.pointCollision = glm::normalize(dif);
        res.penetrationDistance = glm::length(dif) - sphere.radius;
        res.isInCollision = true;
    }
    return res;
}

//TODO TEST
ColliderResult AABBOBBCollision(const Collider& aabb, const Collider& obb, bool dynamicAABB = true) {
    assert(aabb.type == colliderType::AABB);
    assert(obb.type == colliderType::OBB);

    ColliderResult res;
    res.pointCollision = glm::vec3(0);
    res.penetrationDistance = FLT_MAX;
    res.isInCollision = false;

    glm::vec3 test[15] = {
        glm::vec3(1, 0, 0), // AABB axis 1
        glm::vec3(0, 1, 0), // AABB axis 2
        glm::vec3(0, 0, 1), // AABB axis 3
        obb.orientation[0],
        obb.orientation[1],
        obb.orientation[2],
    };

    // Fill out rest of axis
    for (int i = 0; i < 3; ++i) { 
        test[6 + i * 3 + 0] = glm::cross(test[i], test[0]);
        test[6 + i * 3 + 1] = glm::cross(test[i], test[1]);
        test[6 + i * 3 + 2] = glm::cross(test[i], test[2]);
    }

    for (int i = 0; i < 15; ++i) {
        ColliderResult cres = OverlapOnAxisAABBOBB(aabb, obb, test[i], dynamicAABB);
        if (!cres.isInCollision) {
            // Seperating axis found
            return res;
        }else {
            //else if this axe is less distance to answer collision
            if (cres.penetrationDistance < res.penetrationDistance) {
                res.pointCollision = test[i];
                res.penetrationDistance = cres.penetrationDistance;
            }
        }
    }
    // Seperating axis not found
    //We have an intersection here
    res.isInCollision = true;
    return res;
}

//TODO TEST
ColliderResult AABBPlaneCollision(const Collider& aabb, const glm::vec3 normal_plan, float distance_to_origin) {
    assert(aabb.type == colliderType::AABB);

    ColliderResult res;
    res.pointCollision = glm::vec3(0);
    res.penetrationDistance = 0;
    res.isInCollision = false;
    
    // Project the half extents of the AABB onto the plane normal
    float pLen = 
        aabb.size.x * std::abs(normal_plan.x) +
        aabb.size.y * std::abs(normal_plan.y) +
        aabb.size.z * std::abs(normal_plan.z);

    // Find the distance from the center of the AABB to the plane
    float dist = glm::dot(normal_plan, aabb.position) - distance_to_origin;
    // Intersection occurs if the distance falls within the projected side
    if (std::abs(dist) <= pLen) {
        res.isInCollision = true;
        res.penetrationDistance = pLen - std::abs(dist);
        res.pointCollision = (dist<0)? -normal_plan : normal_plan;
    }
    return res;
}

//TODO TEST
ColliderResult OBBOBBCollisionCollision(const Collider& obb1, const Collider& obb2) {
    assert(obb1.type == colliderType::OBB);
    assert(obb2.type == colliderType::OBB);

    ColliderResult res;
    res.pointCollision = glm::vec3(0);
    res.penetrationDistance = 0;
    res.isInCollision = false;

    glm::vec3 test[15] = {
        obb1.orientation[0],
        obb1.orientation[1],
        obb1.orientation[2],
        obb2.orientation[0],
        obb2.orientation[1],
        obb2.orientation[2]
    };

    // Fill out rest of axis
    for (int i = 0; i < 3; ++i) { 
        test[6 + i * 3 + 0] = glm::cross(test[i], test[0]);
        test[6 + i * 3 + 1] = glm::cross(test[i], test[1]);
        test[6 + i * 3 + 2] = glm::cross(test[i], test[2]);
    }

    for (int i = 0; i < 15; ++i) {
        ColliderResult cres = OverlapOnAxisOBBOBB(obb1, obb2, test[i]);
        if (!cres.isInCollision) {
            // Seperating axis found
            return res;
        }
        else {
            //else if this axe is less distance to answer collision
            if (cres.penetrationDistance < res.penetrationDistance) {
                res.pointCollision = test[i];
                res.penetrationDistance = cres.penetrationDistance;
            }
        }
    }
    // Seperating axis not found
    //We have an intersection here
    res.isInCollision = true;
    return res;
}

//TODO TEST
ColliderResult OBBPlaneCollision(const Collider& obb, const glm::vec3 normal_plan, float distance_to_origin) {
    assert(obb.type == colliderType::OBB);

    ColliderResult res;
    res.pointCollision = glm::vec3(0);
    res.penetrationDistance = 0;
    res.isInCollision = false;

    // Project the half extents of the AABB onto the plane normal
    float pLen = 
        obb.size.x * std::abs(glm::dot(normal_plan, obb.orientation[0])) +
        obb.size.y * std::abs(glm::dot(normal_plan, obb.orientation[1])) +
        obb.size.z * std::abs(glm::dot(normal_plan, obb.orientation[2]));
    // Find the distance from the center of the OBB to the plane
    float dist = glm::dot(normal_plan, obb.position) - distance_to_origin;
    // Intersection occurs if the distance falls within the projected side
    
    if (std::abs(dist) <= pLen) {
        res.isInCollision = true;
        res.penetrationDistance = pLen - std::abs(dist);
        res.pointCollision = (dist < 0) ? -normal_plan : normal_plan;
    }
    return res;
}

bool PlanePlaneIntersection(const glm::vec3 normal_plan_1, float distance_to_origin_1, const glm::vec3 normal_plan_2, float distance_to_origin_2) {
    // Compute direction of intersection line
    glm::vec3 d = glm::cross(normal_plan_1, normal_plan_2);

    // Check the length of the direction line
    // if the length is 0, no intersection happened
    return !(compareWithEpsilon(glm::dot(d, d), 0));
}


ColliderResult ColliderSystem::intersect(Collider c1, Collider c2, ColliderResult* res) {
    //Compute collisions depend on the collider type
    if (c1.type == colliderType::AABB && c2.type == colliderType::AABB)     return AABBAABBCollision(c1, c2);
    if (c1.type == colliderType::AABB && c2.type == colliderType::Sphere)   return SphereAABBCollision(c1, c2);
    if (c1.type == colliderType::Sphere && c2.type == colliderType::AABB)   return SphereAABBCollision(c2, c1);
    if (c1.type == colliderType::OBB && c2.type == colliderType::AABB)      return AABBOBBCollision(c2, c1);
    if (c1.type == colliderType::AABB && c2.type == colliderType::OBB)      return AABBOBBCollision(c1, c2);
    if (c1.type == colliderType::Sphere && c2.type == colliderType::Sphere) return SphereSphereCollision(c1, c2);
    if (c1.type == colliderType::Sphere && c2.type == colliderType::OBB)    return SphereOBBCollision(c1, c2);
    if (c1.type == colliderType::OBB && c2.type == colliderType::Sphere)    return SphereOBBCollision(c2, c1);
    if (c1.type == colliderType::OBB && c2.type == colliderType::OBB)       return OBBOBBCollisionCollision(c2, c1);

    ColliderResult res;
    res.isInCollision = false;
    std::cout << "NO COLLISION TEST FOUND !" << std::endl;
    return res;
}
