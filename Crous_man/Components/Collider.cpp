#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <common/ray.hpp>
#include <common/cooldown.hpp>

#include "../InputManager.hpp"
#include "../ECS/EntityManager.hpp"
#include "../ECS/Bitmap.hpp"
#include "../ECS/Entity.hpp"
#include "../Transform.hpp"
#include "../Util.hpp"

#include "Collider.hpp"

#define DEBUG_COLLIDER false


ContactPoint::ContactPoint() {
    this->penetrationDistance = FLT_MAX;
    this->normal = glm::vec3(0, 0, 1);
    this->point = glm::vec3(0);
}

ContactPoint::ContactPoint(ContactPoint* c, bool inversed){
    this->penetrationDistance   = c->penetrationDistance;
    this->normal                = glm::vec3(((inversed)? -c->normal: c->normal));
    this->point                 = glm::vec3(c->point);
}


void ContactPoint::print() {
    std::cout << this->penetrationDistance << " ON p : {" << this->point.x << "," << this->point.y << "," << this->point.z << "}" << "n : {" << this->normal.x << "," << this->normal.y << "," << this->normal.z << "}" << std::endl;
}

ContactPoint::~ContactPoint() {}


ColliderResult::ColliderResult() {
    this->isInCollision = false;
}

ColliderResult::ColliderResult(unsigned short id, ColliderResult* c, bool inversed) {
    this->entityCollidID = id;
    if (c != 0) {
        this->isInCollision = c->isInCollision;
        for (size_t i = 0, size = c->contactsPts.size(); i < size; i++) {
            this->contactsPts.push_back(new ContactPoint(c->contactsPts[i], inversed));
        }
    }
    else {
        this->isInCollision = false;
    }
}

ColliderResult::~ColliderResult() {
    contactsPts.clear();
}

void ColliderResult::print() {
    std::cout << this->isInCollision << std::endl;
    for (size_t i = 0, size = this->contactsPts.size(); i < size; i++) {
        std::cout << "-" << i << " : ";
        this->contactsPts[i]->print();
    }

}

void ColliderResult::inverseNormals() {
    //std::cout << "INVERSE NORMALS" << std::endl;
    //std::cout << this << std::endl;

    for (size_t i = 0, size = this->contactsPts.size(); i < size; i++) {
        //std::cout << "I : " << i << std::endl;
        this->contactsPts[i]->normal = glm::vec3(this->contactsPts[i]->normal) * -1.0f;
    }
}

ColliderSystem::ColliderSystem() : ComponentSystem() {
    requiredComponentsBitmap = new Bitmap({SystemIDs::ColliderID});

    verticesCube.push_back(glm::vec3(-1, -1, 1));
    verticesCube.push_back(glm::vec3(1, -1, 1));
    verticesCube.push_back(glm::vec3(-1, 1, 1));
    verticesCube.push_back(glm::vec3(1, 1, 1));
    verticesCube.push_back(glm::vec3(-1, -1, -1));
    verticesCube.push_back(glm::vec3(1, -1, -1));
    verticesCube.push_back(glm::vec3(-1, 1, -1));
    verticesCube.push_back(glm::vec3(1, 1, -1));
}
ColliderSystem::~ColliderSystem() {

}

void ColliderSystem::initialize(unsigned short i, unsigned short entityID) {
    getCollider(i)->entityID = entityID;

    if (getCollider(i)->type == colliderType::Sphere) {
        getCollider(i)->dimensions = glm::vec3(getCollider(i)->radius);
    }
    if (getCollider(i)->type == colliderType::OBB) {
        getCollider(i)->dimensions = getCollider(i)->size;
    }
    if (getCollider(i)->type == colliderType::AABB) {
        getCollider(i)->orientation = glm::mat3(1);
    }

    addNewColliderEntry(entityID);
}

void ColliderSystem::update(unsigned short i, unsigned short entityID) {
    //We move all the positions from all colliders
    Entity* entity = EntityManager::instance->entities[entityID];
    
    //std::cout << "UPDATE COLLIDER : " << entityID << std::endl;
    //print(entity->worldTransform->translation);

    getCollider(i)->position = entity->worldTransform->translation + getCollider(i)->center;

    //We also need to perform others implementations like dimensions for AABB
    if (getCollider(i)->type == colliderType::Sphere) {
        getCollider(i)->dimensions = glm::vec3(getCollider(i)->radius);
    }
    
    if (getCollider(i)->type == colliderType::AABB) {
        computeAABBDimensions(getCollider(i));
    }

    if (getCollider(i)->type == colliderType::OBB) {
        getCollider(i)->dimensions = getCollider(i)->size;
        getCollider(i)->orientation = EntityManager::instance->entities[entityID]->worldTransform->rotation.rotationMatrix;
    }

    
    if (!isPressedColliderDraw && glfwGetKey(InputManager::instance->window, GLFW_KEY_C) == GLFW_PRESS) {
        isPressedColliderDraw = true;
        drawColliders = !drawColliders;
    }
    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_C) == GLFW_RELEASE) {
        isPressedColliderDraw = false;
    }
    
}

void ColliderSystem::addEntityComponent() {
    EntityManager::instance->colliderComponents.push_back(Collider());
}

void ColliderSystem::renderAll(glm::mat4 view, glm::mat4 projection) {
    if (drawColliders) {
        unsigned short entityID;
        for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
            entityID = entityIDs[i];

            Collider* c = getCollider(i);

            if (!EntityManager::instance->shouldUpdate(entityID) || !c->drawable) {
                continue;
            }

            Entity* e = EntityManager::instance->entities[entityID];
            c->shader->use();

            //std::cout << "RENDER ALL" << std::endl;
            //print(c->orientation);
            //print(c->dimensions);

            glm::mat4 model = glm::mat4(1.0f);
            model[3][0] = c->position.x;
            model[3][1] = c->position.y;
            model[3][2] = c->position.z;
            c->shader->setMVPC(model * glm::mat4(c->orientation), view, projection, isInContactWithSomething(entityID), c->dimensions);
            c->shader->draw();
        }
    }
}

Collider* ColliderSystem::getCollider(unsigned short i) {
    return &EntityManager::instance->colliderComponents[i];
}

Collider* ColliderSystem::getColliderEntityID(unsigned short entityID) {
    for (size_t j = 0, size = entityIDs.size(); j < size; j++) {
        Collider* c = getCollider(j);
        if (c && c->entityID == entityID) return c;
    }
    return 0;
}

bool isSupportedCollider(Collider c) {
    return c.type == colliderType::Sphere || c.type == colliderType::AABB || c.type == colliderType::OBB;
}

void ColliderSystem::addNewColliderEntry(unsigned short entityID) {
    //std::cout << "ADD NEW COMPOSANT ENTRY : " << entityID << std::endl;
    //Resize older components (for perform add in game)
    for (auto const& x : collisionResultMap) {
        if (x.first != (unsigned short)-1 && collisionResultMap.at(x.first).size() < entityIDs.size()) {

            //std::cout << "NEED TO RESIZE : " << x.first << std::endl;
            collisionResultMap.at(x.first).resize(entityIDs.size(), 0);
        
        }
    }

    //for (auto const& x : simpleCollisionResultMap) {
    //    if (simpleCollisionResultMap.at(x.first).size() < entityIDs.size()) {
    //
    //        std::cout << "NEED TO RESIZE : " << x.first << std::endl;
    //        simpleCollisionResultMap.at(x.first).resize(entityIDs.size(), false);
    //
    //    }
    //}

    //Init collider datas
    std::vector<ColliderResult*> res;
    res.resize(entityIDs.size(), 0);
    collisionResultMap.insert(CollisionResultMap::value_type(entityID, res));

    //std::vector<bool> res_simple;
    //res_simple.resize(entityIDs.size(), false);
    //simpleCollisionResultMap.insert(SimpleCollisionResultMap::value_type(entityID, res_simple));
}

/* =============== Methods for collision detection and resolution =============== */
void ColliderSystem::updateCollision(unsigned short i, unsigned short entityID) {
    unsigned short entityIDJ;
    
    for (size_t j = i + 1, size = entityIDs.size(); j < size; j++) {
        entityIDJ = entityIDs[j];
        if (EntityManager::instance->shouldUpdate(entityID) && EntityManager::instance->shouldUpdate(entityIDJ)) {
            computeIntersection(i, entityID, j, entityIDJ);
        }else{
            if (entityID != (unsigned short) -1) {
                //Need to clear collision jic
                //simpleCollisionResultMap.at(entityID)[j] = false;
                collisionResultMap.at(entityID)[j] =  0;
            }

            if (entityIDJ != (unsigned short) -1) {
                //Could be optional
                //simpleCollisionResultMap.at(entityIDJ)[i] = false;
                collisionResultMap.at(entityIDJ)[i] =  0;
            }
        }
    }
}

void ColliderSystem::updateOnCollide(unsigned short i, unsigned short entityID) {
    //For perform physics implementations next
}

void ColliderSystem::updateCollisionAll() {
    //std::cout << "UPDATE COLLISION ALL" << std::endl;
    ComponentSystem::updateCollisionAll();
    //logCollisionResultMap(collisionResultMap);
}

void ColliderSystem::updateOnCollideAll() {
    ComponentSystem::updateOnCollideAll();
    //logCollisionResultMap(collisionResultMap);
}

bool ColliderSystem::isInContactWithSomething(unsigned short entityID) {
    for (size_t j = 0; j < entityIDs.size(); j++){
        //if (simpleCollisionResultMap.at(entityID)[j]) return true;
        ColliderResult* r = collisionResultMap.at(entityID)[j];
        if (r) {
            if(r->isInCollision) return true;
        }
    }

    return false;
}

void logCollisionResultMap(CollisionResultMap m) {
    std::cout << "LENGTH ELEM : " << m.size() << std::endl;
    for (auto const& x : m) {
        std::cout << x.first << " : [" << std::endl;
        for (auto const& el : x.second) {
            if (el) {
                el->print();
            }
            else {
                std::cout << 0 << std::endl;
            }
        }
        std::cout << "]" << std::endl;
    }
}
void logSimpleCollisionResultMap(SimpleCollisionResultMap m) {
    std::cout << "LENGTH ELEM : " << m.size() << std::endl;
    for (auto const& x : m) {
        std::cout << x.first << " : [" << std::endl;
        for (auto const& el : x.second) {
            std::cout << el  << "," << std::endl;
        }
        std::cout << "]" << std::endl;
    }
}


void ColliderSystem::clearAllCollision(unsigned short i) {
    for (size_t j = 0, size = entityIDs.size(); j < size; j++) {
        collisionResultMap.at(i)[j] = 0;
        //simpleCollisionResultMap.at(i)[j] = false;
    }
}


/* =================== =================== COLLISION DETECTION FUNCTIONS =================== ===================*/
//create AABB form a min and a max position only
void AABBfromMinMax(Collider& aabb, const glm::vec3 min, const glm::vec3 max) {
    aabb.type = colliderType::AABB;
    aabb.position = (min + max) * 0.5f;
    aabb.size = (max - min) * 0.5f;
}

//Compute AABB dimension value
void ColliderSystem::computeAABBDimensions(Collider* aabb) {
    //We need to take the posible rotation in care here
    glm::mat3 rot = EntityManager::instance->entities[aabb->entityID]->worldTransform->rotation.rotationMatrix;

    glm::vec3 v1 = rot[0] * aabb->size[0];
    glm::vec3 v2 = rot[1] * aabb->size[1];
    glm::vec3 v3 = rot[2] * aabb->size[2];
    
    glm::vec3 min = v1* verticesCube[0].x+ v2 * verticesCube[0].y+ v3 * verticesCube[0].z;
    glm::vec3 max = v1* verticesCube[0].x+ v2 * verticesCube[0].y+ v3 * verticesCube[0].z;
    for (size_t i = 1; i < 8; i++){
       glm::vec3 vertex = v1 * verticesCube[i].x + v2 * verticesCube[i].y + v3 * verticesCube[i].z;
       minVec3(vertex,min,min);
       maxVec3(vertex,max,max);
    }

   aabb->dimensions = (max - min)/2.0f;
}
void computeMinMaxAABB(const Collider& aabb, glm::vec3& min, glm::vec3& max) {
    assert(aabb.type == colliderType::AABB);
    min = -aabb.dimensions - glm::vec3(FLT_EPSILON);
    max = aabb.dimensions + glm::vec3(FLT_EPSILON);
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
    min += aabb.position;
    max += aabb.position;

    //std::cout << "POINT IN AABB" << std::endl;
    //std::cout << "MIN: ";
    //print(min);
    //std::cout << "MAX: ";
    //print(max);
    //std::cout << "POINT: ";
    //print(point);

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

    //std::cout << " ++++ BTEST POINT IN OBB" << std::endl;

    glm::vec3 dir = point - obb.position;
    //print(point);
    //print(obb.position);
    //print(obb.size);
    //print(dir);

    for (int i = 0; i < 3; ++i) {
        //std::cout << "  ========= AXIS TEST : " << i << std::endl;
        glm::vec3 axis = obb.orientation[i];

        //print(axis);

        float distance = glm::dot(dir, axis);
        //print(distance);

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


//Find closest point between collider and another collider position
glm::vec3 closestPointSphere(const Collider& sphere, const glm::vec3& point) {
    assert(sphere.type == colliderType::Sphere);

    glm::vec3 sphereToPoint = glm::normalize(point - sphere.position);
    sphereToPoint = sphereToPoint * sphere.radius;
    return sphereToPoint + sphere.position;
}
          
glm::vec3 closestPointAABB(const Collider& aabb, const glm::vec3& point) {
    assert(aabb.type == colliderType::AABB);
    //std::cout << "CLOSEST PTN AABB" << std::endl;

    glm::vec3 result = point;
    glm::vec3 min;
    glm::vec3 max;
    computeMinMaxAABB(aabb, min, max);
    min += aabb.position;
    max += aabb.position;

    //print(min);
    //print(max);
    //print(point);

    //Clamp to min
    result.x = (result.x < min.x) ? min.x : result.x;
    result.y = (result.y < min.y) ? min.y : result.y;
    result.z = (result.z < min.z) ? min.z : result.z;

    //Clamp to max
    result.x = (result.x > max.x) ? max.x : result.x;
    result.y = (result.y > max.y) ? max.y : result.y;
    result.z = (result.z > max.z) ? max.z : result.z;

    //print(result);
    //std::cout << " zzzz CLOSEST PTN AABB" << std::endl;
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



//Method resolving axis overlapping with interval structure
typedef struct Interval {
    float min;
    float max;
} Interval;

void print(Interval& i) {
    std::cout << "min : " << i.min << std::endl;
    std::cout << "max : " << i.max << std::endl;
}

Interval GetIntervalOBB(const Collider& obb, const glm::vec3& axis) {
    assert(obb.type == colliderType::OBB);
    //std::cout << "GET INTERVALE OBB" << std::endl;
    //print(obb.size);
    //print(obb.position);
    //print(obb.orientation);
    //print(obb.center);
    //print(obb.dimensions);


    glm::vec3 vertex[8];

    glm::vec3 C = obb.position;	// OBB Center
    glm::vec3 E = obb.size;     // OBB Extents


    //obb.orientation[0] => col 0 (first axis of the rotation)
    //Find the actual vertices world position
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

    computeMinMaxAABB(aabb, i, a);
    //std::cout << "CPT MIN MAX : " << std::endl;
    //print(i);
    //print(a);
    //print(aabb.position);

    i = aabb.position + i;
    a = aabb.position + a;

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


/* =================== COLLIDERS COLLISION DETECTION FUNCTIONS ===================*/
//Collision between two AABB
void AABBAABBCollision(const Collider& aabb1, const Collider& aabb2, ColliderResult* res) {
    //std::cout << "INTERSECTION AABB AABB" << std::endl;
    assert(aabb1.type == colliderType::AABB);
    assert(aabb2.type == colliderType::AABB);
    assert(res != 0);

    ContactPoint* cp_res = 0;
   
    glm::vec3 min_1;
    glm::vec3 min_2;
    glm::vec3 max_1;
    glm::vec3 max_2;
    computeMinMaxAABB(aabb1, min_1, max_1);
    computeMinMaxAABB(aabb2, min_2, max_2);

    bool overlap_x = ((aabb1.position.x + min_1.x) < (aabb2.position.x + max_2.x)) && ((aabb2.position.x + min_2.x) < (aabb1.position.x + max_1.x));
    bool overlap_y = ((aabb1.position.y + min_1.y) < (aabb2.position.y + max_2.y)) && ((aabb2.position.y + min_2.y) < (aabb1.position.y + max_1.y));
    bool overlap_z = ((aabb1.position.z + min_1.z) < (aabb2.position.z + max_2.z)) && ((aabb2.position.z + min_2.z) < (aabb1.position.z + max_1.z));

    bool incontact = overlap_x && overlap_y && overlap_z;
    if (incontact) {
        cp_res = new ContactPoint();
        //If the two colliders are in contact
        //try to find the min distance to split them

        float dist_x_1 = (aabb1.position.x + max_1.x) - (aabb2.position.x + min_2.x);
        float dist_x_2 = (aabb2.position.x + max_2.x) - (aabb1.position.x + min_1.x);

        float dist_y_1 = (aabb1.position.y + max_1.y) - (aabb2.position.y + min_2.y);
        float dist_y_2 = (aabb2.position.y + max_2.y) - (aabb1.position.y + min_1.y);

        float dist_z_1 = (aabb1.position.z + max_1.z) - (aabb2.position.z + min_2.z);
        float dist_z_2 = (aabb2.position.z + max_2.z) - (aabb1.position.z + min_1.z);


        float x_factor = (dist_x_1 < dist_x_2) ? -1 : 1;
        float y_factor = (dist_y_1 < dist_y_2) ? -1 : 1;
        float z_factor = (dist_z_1 < dist_z_2) ? -1 : 1;

        float x = ((dist_x_1 < dist_x_2) ? dist_x_1 : dist_x_2) * 0.5;
        float y = ((dist_y_1 < dist_y_2) ? dist_y_1 : dist_y_2) * 0.5;
        float z = ((dist_z_1 < dist_z_2) ? dist_z_1 : dist_z_2) * 0.5;

        if (x <= z && x <= y) {
            cp_res->penetrationDistance = x;
            cp_res->normal = glm::vec3(x_factor,0,0);
        }
        else if (y <= x && y <= z) {
            cp_res->penetrationDistance = y;
            cp_res->normal = glm::vec3(0, y_factor, 0);
        }
        else if (z <= y && z <= x) {
            cp_res->penetrationDistance = z;
            cp_res->normal = glm::vec3(0, 0, z_factor);
        }

        //To compute the point, we chose the mid point for the two projections point on both AABB
        cp_res->point.x += ((x_factor < 0) ? max_1.x - x : min_1.x + x);
        cp_res->point.y += ((y_factor < 0) ? max_1.y - y : min_1.y + y);
        cp_res->point.z += ((z_factor < 0) ? max_1.z - z : min_1.z + z);
    }

    //If there is a contact point
    if (cp_res != 0) {
        res->isInCollision = true;
        res->contactsPts.push_back(cp_res);
    }
}

//Collision between two Sphere
void SphereSphereCollision(Collider sp1, Collider sp2, ColliderResult* res) {
    if (DEBUG_COLLIDER) {
        std::cout << "INTERSECTION SPHERE SPHERE" << std::endl;
    }
        
    assert(sp1.type == colliderType::Sphere);
    assert(sp2.type == colliderType::Sphere);
    assert(res != 0);

    float r = sp1.radius + sp2.radius;
    ContactPoint* cp_res = 0;

    glm::vec3 mvt = sp1.position - sp2.position;

    float distance = glm::length(mvt);

    if (DEBUG_COLLIDER) {
        std::cout << "DISTANCE : " << distance << std::endl;
        std::cout << "RAYON : " << r << std::endl;
    }

    if (distance < r) {
        if (DEBUG_COLLIDER) {
            std::cout << "CONTACT !!" << std::endl;
        }
        cp_res = new ContactPoint();
        
        //res->penetrationDistance = (sp1.radius - distance) + sp2.radius;
        cp_res->penetrationDistance = (distance - r) *0.5f;
        cp_res->normal = ((distance == 0) ? glm::vec3(0, 1, 0) : glm::normalize(mvt));

        float distToInterP = sp1.radius - cp_res->penetrationDistance;
        cp_res->point = sp1.position + (cp_res->normal * distToInterP);
    }

    //If there is a contact point
    if (cp_res != 0) {
        if (DEBUG_COLLIDER) {
            std::cout << "CP TROUVE !"<< std::endl;
        }
        res->isInCollision = true;
        cp_res->print();

        res->contactsPts.push_back(cp_res);
    }
    if (DEBUG_COLLIDER) {
        std::cout << "=========== END ==========" << std::endl;
    }
}


//Collision between an AABB and a sphere
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
ColliderResult* SphereAABBCollision_old(const Collider& aabb, const Collider& sphere) {
    assert(aabb.type == colliderType::AABB);
    assert(sphere.type == colliderType::Sphere);
    

    ColliderResult* res = new ColliderResult();
    ContactPoint* cp_res = new ContactPoint();
    
    glm::vec3 min;
    glm::vec3 max;
    computeMinMaxAABB(aabb, min, max);

    glm::vec3 difference = sphere.position - aabb.position;

    //Get the coord of the point in the sphere at smalest dist from the cube
    float x = std::max(min.x, std::min(difference.x, max.x));
    float y = std::max(min.y, std::min(difference.y, max.y));
    float z = std::max(min.z, std::min(difference.z, max.z));

    difference = (glm::vec3(x, y, z) + aabb.position) - sphere.position;
    float distance = glm::length(difference);

    if (distance < sphere.radius) {
        //We got a contact
        res->isInCollision = true;

        //If both are in the same place
        if (distance == 0) {
            //Get min dist to get out of other collider
            glm::vec3 dist = max - min;
            cp_res->point = sphere.position;


            if (dist.y <= dist.x && dist.y <= dist.z) {
                cp_res->penetrationDistance = (dist.y + sphere.radius) * .5;
                cp_res->normal = glm::vec3(0, 1, 0);
            }
            else if (dist.x <= dist.y && dist.x <= dist.z) {
                cp_res->penetrationDistance = (dist.x + sphere.radius) * .5;
                cp_res->normal = glm::vec3(1, 0, 0);
            }
            else if (dist.z <= dist.y && dist.z <= dist.x) {
                cp_res->penetrationDistance = (dist.z + sphere.radius)* .5;
                cp_res->normal = glm::vec3(0, 0, 1);
            }
        }
        else {
            //We need to know the direction of the minimum penetration
            cp_res->normal = perform_direction(difference, cp_res->penetrationDistance, sphere.radius);
            //We also need to know point to apply normal and distance
        }

        res->contactsPts.push_back(cp_res);
    }
    return res;
}

void SphereAABBCollision(const Collider& aabb, const Collider& sphere, ColliderResult* res) {
    //std::cout << "INTERSECTION SPHERE AABB" << std::endl;
    assert(aabb.type == colliderType::AABB);
    assert(sphere.type == colliderType::Sphere);
    assert(res != 0);

    ContactPoint* cp_res = 0;

    glm::vec3 closestPoint = closestPointAABB(aabb, sphere.position);

    glm::vec3 dif = sphere.position - closestPoint;
    float dist_squared = glm::dot(dif,dif);
    float radius_squared = sphere.radius * sphere.radius;

    if (dist_squared < radius_squared) {
        //std::cout << "INTERSECTION !" << std::endl;
        //print(closestPoint);
        //print(dif);

        cp_res = new ContactPoint();
        //Collision !
        glm::vec3 normal;

        //closest point correspond to sphere center
        if (compareWithEpsilon(dist_squared, 0.0f)) {
            glm::vec3 dif_aabb = closestPoint - aabb.position;
            
            if (compareWithEpsilon(glm::dot(dif_aabb, dif_aabb), 0.0f)) {
                //Both are on the same place, we can then propulse from one of the center trougth any direction with penetration of rad+mindim/2
                cp_res->point = aabb.position;

                glm::vec3 dist = aabb.dimensions * 2.0f;

                if (dist.y <= dist.x && dist.y <= dist.z) {
                    cp_res->penetrationDistance = (dist.y + sphere.radius) * .5;
                    cp_res->normal = glm::vec3(0, 1, 0);
                }
                else if (dist.x <= dist.y && dist.x <= dist.z) {
                    cp_res->penetrationDistance = (dist.x + sphere.radius) * .5;
                    cp_res->normal = glm::vec3(1, 0, 0);
                }
                else if (dist.z <= dist.y && dist.z <= dist.x) {
                    cp_res->penetrationDistance = (dist.z + sphere.radius) * .5;
                    cp_res->normal = glm::vec3(0, 0, 1);
                }
                
                res->isInCollision = true;
                res->contactsPts.push_back(cp_res);
                return;
            }

            // Closest point is at the center of the sphere
            normal = glm::normalize(dif_aabb);
        }
        else {
            normal = glm::normalize(dif);
        }

        glm::vec3 outsidePoint = sphere.position - normal * sphere.radius;

        float distance = glm::length(closestPoint - outsidePoint);

        cp_res->point = closestPoint + (outsidePoint - closestPoint) * 0.5f;
        cp_res->normal = normal;
        cp_res->penetrationDistance = distance * 0.5f;
    }

    //If there is a contact point
    if (cp_res != 0) {
        res->isInCollision = true;
        res->contactsPts.push_back(cp_res);
    }
}


//Collision between an OBB and a sphere
void OBBSphereCollision(const Collider& sphere, const Collider& obb, ColliderResult* res) {
    //std::cout << "INTERSECTION SPHERE OBB" << std::endl;
    assert(sphere.type == colliderType::Sphere);
    assert(obb.type == colliderType::OBB);
    assert(res != 0);

    ContactPoint* cp_res = 0;
    
    glm::vec3 closestPoint = closestPointOBB(obb, sphere.position);
    

    //std::cout << "  => SPHERE POSITION : ";
    //print(sphere.position);
    //std::cout << "  => SPHERE closestPoint : ";
    //print(closestPoint);

    //glm::vec3 dif = closestPoint - sphere.position;
    glm::vec3 dif = sphere.position - closestPoint;

    //std::cout << "  => DIF : ";
    //print(dif); 

    float dist_squared = glm::dot(dif,dif);
    //std::cout << "DIST SQUARED : " << dist_squared << std::endl;
    float radius_squared = sphere.radius * sphere.radius;

    if (dist_squared < radius_squared) {
        //We got a collision then

        //std::cout << "WE GOT A COLLISION" << std::endl;
        cp_res = new ContactPoint();
        glm::vec3 normal;

        if (compareWithEpsilon(dist_squared, 0.0f)) {
            //std::cout << " + DIST SQUARED NULL" << std::endl;

            glm::vec3 dif_obb = closestPoint - obb.position;

            if (compareWithEpsilon(glm::dot(dif_obb, dif_obb), 0.0f)) {
                //Both are in the same area, we can move the Sphere on the smalest dimension of the OBB
                cp_res->point = sphere.position;
                
                Interval i_x =  GetIntervalOBB(obb, glm::vec3(1,0,0));
                Interval i_y =  GetIntervalOBB(obb, glm::vec3(0,1,0));
                Interval i_z =  GetIntervalOBB(obb, glm::vec3(0,0,1));
                
                glm::vec3 dist = glm::vec3(i_x.max - i_x.min, i_y.max - i_y.min, i_z.max - i_z.min);
                if (dist.y <= dist.x && dist.y <= dist.z) {
                    cp_res->penetrationDistance = (dist.y + sphere.radius) * .5;
                    cp_res->normal = glm::vec3(0,1,0) * obb.orientation;
                }
                else if (dist.x <= dist.y && dist.x <= dist.z) {
                    cp_res->penetrationDistance = (dist.x + sphere.radius) * .5;
                    cp_res->normal = glm::vec3(1, 0, 0) * obb.orientation;
                }
                else if (dist.z <= dist.y && dist.z <= dist.x) {
                    cp_res->penetrationDistance = (dist.z + sphere.radius) * .5;
                    cp_res->normal = glm::vec3(0, 0, 1) * obb.orientation;
                }

                res->isInCollision = true;
                res->contactsPts.push_back(cp_res);
                return;
            }

            // Closest point is at the center of the sphere
            normal = glm::normalize(dif_obb);
        }else {
            //std::cout << " + DIST SQUARED NOT NULL" << std::endl;
            normal = glm::normalize(dif);
        }

        //std::cout << "NORMAL : ";
        //print(normal);

        glm::vec3 outsidePoint = sphere.position - normal * sphere.radius;

        float distance = glm::length(closestPoint - outsidePoint);

        cp_res->point = closestPoint + (outsidePoint - closestPoint) * 0.5f;
        cp_res->normal = normal;
        cp_res->penetrationDistance = distance * 0.5f;
    }


    //If there is a contact point
    if (cp_res != 0) {
        res->isInCollision = true;
        res->contactsPts.push_back(cp_res);
    }

    //std::cout << "========== END ==========" << std::endl;
}

//Collision between a plane and a sphere
//Plane is discribed by its normal and a distance to origin
void SpherePlaneCollision(const Collider& sphere, const glm::vec3 normal_plan, float distance_to_origin, ColliderResult* res) {
    //std::cout << "INTERSECTION SPHERE PLANE" << std::endl;
    assert(sphere.type == colliderType::Sphere);
    assert(res != 0);

    ContactPoint* cp_res = 0;

    glm::vec3 closestPoint = closestPointPlane(sphere.position, normal_plan,distance_to_origin);
    glm::vec3 dif = sphere.position - closestPoint;
    float dist_squared = glm::dot(dif, dif);
    float radius_squared = sphere.radius * sphere.radius;

    if (dist_squared < radius_squared) {
        cp_res = new ContactPoint();
        //We got a collision then       
        cp_res->normal = glm::normalize(dif);
        cp_res->point = closestPoint;
        cp_res->penetrationDistance = (glm::length(dif) - sphere.radius) * 0.5;
     }

    //If there is a contact point
    if (cp_res != 0) {
        res->isInCollision = true;
        res->contactsPts.push_back(cp_res);
    }
}

//Collision between an AABB and an OBB
//Get all the vertices of an AABB
//OK
void getVerticesAABB(const Collider& aabb, std::vector<glm::vec3>& vertex) {
    assert(aabb.type == colliderType::AABB);
    vertex.reserve(8);
    vertex.push_back(aabb.position + glm::vec3( -1   , -1,  1)   * aabb.dimensions);
    vertex.push_back(aabb.position + glm::vec3(  1    ,-1,  1)    * aabb.dimensions);
    vertex.push_back(aabb.position + glm::vec3( -1   ,  1,  1)    * aabb.dimensions);
    vertex.push_back(aabb.position + glm::vec3(  1    , 1,  1)     * aabb.dimensions);
    vertex.push_back(aabb.position + glm::vec3( -1   , -1, -1)  * aabb.dimensions);
    vertex.push_back(aabb.position + glm::vec3(  1    ,-1, -1)   * aabb.dimensions);
    vertex.push_back(aabb.position + glm::vec3( -1   ,  1, -1)   * aabb.dimensions);
    vertex.push_back(aabb.position + glm::vec3(  1    , 1, -1)    * aabb.dimensions);
}

//Get all the edges of an AABB
//OK
void getEdgesAABB(const Collider& aabb, std::vector<glm::vec3>& start_lines, std::vector<glm::vec3>& end_lines) {
    assert(aabb.type == colliderType::AABB);
    
    start_lines.reserve(12);
    end_lines.reserve(12);

    std::vector<glm::vec3> v;
    getVerticesAABB(aabb, v);

    int index[][2] = { // Indices of edges
        { 0, 1 },{ 2, 3 },{ 0, 2 },{ 1, 3 },{ 3, 7 },{ 1, 5 },
        { 4, 5 },{ 4, 6 },{ 6, 7 },{ 7, 5 },{ 6, 2 },{ 4, 0 }
    };
    for (int j = 0; j < 12; ++j) {
        start_lines.push_back(v[index[j][0]]);
        end_lines.push_back(v[index[j][1]]);
    }
}

//Get all the planes of an AABB
//OK
void getPlanesAABB(const Collider& aabb, std::vector<glm::vec3>& normals_plane, std::vector<float>& distances_to_origin) {
    assert(aabb.type == colliderType::AABB);

    glm::vec3 c = aabb.position;	
    glm::vec3 e = aabb.dimensions;

    glm::vec3 a[] = {			
        glm::vec3(1,0,0),
        glm::vec3(0,1,0),
        glm::vec3(0,0,1),
    };

    normals_plane.resize(6);
    distances_to_origin.resize(6);

    normals_plane[0] = a[0];
    normals_plane[1] = a[0] * -1.0f;
    normals_plane[2] = a[1];
    normals_plane[3] = a[1] * -1.0f;
    normals_plane[4] = a[2];
    normals_plane[5] = a[2] * -1.0f;

    distances_to_origin[0] = glm::dot(a[0], (c + a[0] * e.x));
    distances_to_origin[1] = -glm::dot(a[0], (c - a[0] * e.x));
    distances_to_origin[2] = glm::dot(a[1], (c + a[1] * e.y));
    distances_to_origin[3] = -glm::dot(a[1], (c - a[1] * e.y));
    distances_to_origin[4] = glm::dot(a[2], (c + a[2] * e.z));
    distances_to_origin[5] = -glm::dot(a[2], (c - a[2] * e.z));
}

//The edges provided are clipped against the planes of the provided bounding box
std::vector<glm::vec3> clipEdgesToAABB(const std::vector<glm::vec3>& start_edges, const std::vector<glm::vec3>& end_edges, const Collider& aabb) {
    assert(aabb.type == colliderType::AABB);

    std::vector<glm::vec3> result;
    result.reserve(start_edges.size() * 3);

    glm::vec3 intersection;

    std::vector<glm::vec3> normals_plane;
    std::vector<float> distances_to_origin;
    getPlanesAABB(aabb, normals_plane, distances_to_origin);

    //std::cout << " CLIP EDGES TO AABB" << std::endl;
    //for (size_t i = 0; i < normals_plane.size(); i++){
    //    
    //    print(normals_plane[i]);
    //    print(distances_to_origin[i]);
    //}


    for (int i = 0; i < normals_plane.size(); ++i) {

        for (int j = 0; j < start_edges.size(); ++j) {
            if (clipToPlane(normals_plane[i], distances_to_origin[i], start_edges[j], end_edges[j], &intersection)) {
                
                //std::cout << " => normal : ";
                //print(normals_plane[i]);
                //std::cout << "DISTANCE TO ORIGIN : " << distances_to_origin[i] << std::endl;
                //std::cout << " start edge : ";
                //print(start_edges[j]);
                //std::cout << " end edge : ";
                //print(end_edges[j]);
                //std::cout << " RES : ";
                //print(intersection);

                
                if (pointInAABB(intersection, aabb)) {
                    //std::cout <<"    = >PUSHED : ";
                    //print(intersection);
                    result.push_back(intersection);
                }
            }
        }
    }

    return result;
}

float penetrationDepthAABBOBB(const Collider& aabb, const Collider& obb, const glm::vec3& axis, bool* outShouldFlip) {
    glm::vec3 axisn = glm::normalize(axis);

    Interval i1 = GetIntervalAABB(aabb, axisn);
    Interval i2 = GetIntervalOBB(obb, axisn);

    if (!((i2.min <= i1.max) && (i1.min <= i2.max))) {
        return 0.0f; // No penerattion
    }

    float len1 = i1.max - i1.min;
    float len2 = i2.max - i2.min;

    float min = std::min(i1.min, i2.min);
    float max = std::max(i1.max, i2.max);
    float length = max - min;

    if (outShouldFlip != 0) {
        *outShouldFlip = (i2.min < i1.min);
    }
    return (len1 + len2) - length;
}

void AABBOBBCollision(const Collider& aabb, const Collider& obb, ColliderResult* res) {
    //std::cout << "=================== INTERSECTION OBB AABB" << std::endl;
    assert(aabb.type == colliderType::AABB);
    assert(obb.type == colliderType::OBB);
    assert(res != 0);

    //SAT basic treatment
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
        test[6 + i * 3 + 0] = glm::cross(test[i], test[3]);
        test[6 + i * 3 + 1] = glm::cross(test[i], test[4]);
        test[6 + i * 3 + 2] = glm::cross(test[i], test[5]);
    }


    glm::vec3* hitNormal = 0;
    bool shouldFlip;
    float min_pen_dist = FLT_MAX;

    for (int i = 0; i < 15; ++i) {
        //No need to treat zero sized vectors
        if (glm::dot(test[i], test[i]) < 0.001f) {
            //std::cout << "ELIMINATED VECTOR" << std::endl;
            //print(test[i]);
            continue;
        }

        float depth = penetrationDepthAABBOBB(aabb, obb, test[i], &shouldFlip);
        if (depth <= 0.0f) {
            //We found a separation axis
            //std::cout << "SAT FOUND" << std::endl;
            //print(test[i]);
            return;
        }

        else if (depth < min_pen_dist) {
            if (shouldFlip) {
                test[i] = test[i] * -1.0f;
            }
            min_pen_dist = depth;
            hitNormal = &test[i];
        }
    }

    if (hitNormal == 0) {
        //std::cout << "NO AXIS FOUND" << std::endl;
        //No axis found 
        return;
    }

    //So we got an intersection then
    res->isInCollision = true;

    //The axis with the minimum penetration
    //The axis with the minimum displacmeent to perform for avoiding the collision
    glm::vec3 axis = glm::normalize(*hitNormal);

    std::vector<glm::vec3> start_edges_2;
    std::vector<glm::vec3> end_edges_2;

    std::vector<glm::vec3> start_edges_1;
    std::vector<glm::vec3> end_edges_1;

    getEdgesAABB(aabb, start_edges_1, end_edges_1);
    getEdgesOBB(obb, start_edges_2, end_edges_2);

    //std::cout << "NB sedge AABB : " << start_edges_1.size() << std::endl;
    //std::cout << "NB eedge AABB : " << end_edges_1.size() << std::endl;
    //for (size_t i = 0; i < start_edges_1.size(); i++) {
    //    std::cout << "EDGE I : " << i << std::endl;
    //    print(start_edges_1[i]);
    //    print(end_edges_1[i]);
    //
    //    print(start_edges_2[i]);
    //    print(end_edges_2[i]);
    //}
    //std::cout << "NB sedge OBB : " << start_edges_2.size() << std::endl;
    //std::cout << "NB eedge OBB : " << end_edges_2.size() << std::endl;
    //std::cout << "AXIS : " << std::endl;
    //print(axis);
    //std::cout << "penetration : " << min_pen_dist << std::endl;


    std::vector<glm::vec3> c1 = clipEdgesToAABB(start_edges_2, end_edges_2, aabb);
    std::vector<glm::vec3> c2 = clipEdgesToOBB(start_edges_1, end_edges_1, obb);

    //std::cout << "NB C1 : " << c1.size() << std::endl;
    //std::cout << "NB C2 : " << c2.size() << std::endl;

    //All possible contact points
    std::vector<glm::vec3> pContacts;
    pContacts.reserve(c1.size() + c2.size());
    pContacts.insert(pContacts.end(), c1.begin(), c1.end());
    pContacts.insert(pContacts.end(), c2.begin(), c2.end());


    if (pContacts.size() == 0) {
        //std::cout << "========== NO CONTACT POINT ==========" << std::endl;

        //If we dont have any point, its because one obb is in the other one
        ContactPoint* cp_res = new ContactPoint();
        cp_res->normal = axis;
        cp_res->penetrationDistance = min_pen_dist;
        //First find the box inside the other
        if (pointInAABB(obb.position, aabb)) {
            //OBB in AABB
            //Find the closest point in OBB
            cp_res->point = closestPointOBB(obb, aabb.position);
        }
        else {
            //AABB in OBB
            cp_res->point = closestPointAABB(aabb, obb.position);
        }
        res->contactsPts.push_back(cp_res);
    }
    else {
        //std::cout << "========== CONTACT POINT ==========" << std::endl;

        //Find the OBB interval on the current chosed axis 
        //Interval i = GetIntervalOBB(obb, axis);

        Interval i = GetIntervalAABB(aabb, axis);

        //std::cout << "+++ GET INTERVAL : " << std::endl;
        //print(axis);
        //print(obb.position);
        //print(obb.dimensions);
        //std::cout << "min : " << i.min << std::endl;
        //std::cout << "max : " << i.max << std::endl;
        //std::cout << "MIN PEN DIST : " << min_pen_dist << std::endl;

        float distance = (i.max - i.min) * 0.5f - min_pen_dist * 0.5f;
        //float distance = i.min + min_pen_dist;
        glm::vec3 pointOnPlane = aabb.position + axis * distance;

        //std::cout << "DISTANCE : " << distance << std::endl;
        //print(pointOnPlane);

        //Projecting the result of the clipped points onto a shared plane. 
        //The shared plane is constructed out of the collision normal
        //std::cout << "TAILLE PCONT : " << pContacts.size() << std::endl;
        for (int i = pContacts.size() - 1; i >= 0; --i) {
            glm::vec3 contact = pContacts[i];
            pContacts[i] = contact + (axis * glm::dot(axis, pointOnPlane - contact));

            //std::cout << i << std::endl;
            //print(pContacts[i]);

            //Erase similar contact points
            for (int j = pContacts.size() - 1; j > i; --j) {
                glm::vec3 dif = pContacts[j] - pContacts[i];
                if (glm::dot(dif, dif) < 0.0001f) {
                    pContacts.erase(pContacts.begin() + j);
                    break;
                }
            }
        }


        if (pContacts.size() > 0) {
            for (int i = 0; i < pContacts.size(); i++) {
                ContactPoint* cp_res = new ContactPoint();
                cp_res->normal = axis;
                cp_res->point = pContacts[i];
                cp_res->penetrationDistance = min_pen_dist;

                res->contactsPts.push_back(cp_res);
            }
        }
    }
}

//Collision between an AABB and a Plane
//Plane is discribed by its normal and a distance to origin
void AABBPlaneCollision(const Collider& aabb, const glm::vec3 normal_plan, float distance_to_origin, ColliderResult* res) {
    //std::cout << "INTERSECTION AABB PLANE" << std::endl;
    assert(aabb.type == colliderType::AABB);
    assert(res != 0);

    ContactPoint* cp_res = 0;
    
    // Project the half extents of the AABB onto the plane normal
    float pLen = 
        aabb.dimensions.x * std::abs(normal_plan.x) +
        aabb.dimensions.y * std::abs(normal_plan.y) +
        aabb.dimensions.z * std::abs(normal_plan.z);

    // Find the distance from the center of the AABB to the plane
    float dist = glm::dot(normal_plan, aabb.position) - distance_to_origin;
    // Intersection occurs if the distance falls within the projected side
    if (std::abs(dist) <= pLen) {
        cp_res = new ContactPoint();
        cp_res->penetrationDistance = pLen - std::abs(dist);
        cp_res->normal = (dist<0)? -normal_plan : normal_plan;
        cp_res->point = aabb.position + (cp_res->normal * (std::abs(dist) - cp_res->penetrationDistance));
    }

    //If there is a contact point
    if (cp_res != 0) {
        res->isInCollision = true;
        res->contactsPts.push_back(cp_res);
    }
}

//Collision between two OBB
//Get all the vertices of an OBB
void getVerticesOBB(const Collider& obb, std::vector<glm::vec3>& vertex) {
    assert(obb.type == colliderType::OBB);

    vertex.resize(8);
    glm::vec3 C = obb.position;	// OBB Center
    glm::vec3 E = obb.size;     // OBB Extents

    //Find the actual vertices world position
    vertex[0] = C + obb.orientation[0] * E[0] + obb.orientation[1] * E[1] + obb.orientation[2] * E[2];
    vertex[1] = C - obb.orientation[0] * E[0] + obb.orientation[1] * E[1] + obb.orientation[2] * E[2];
    vertex[2] = C + obb.orientation[0] * E[0] - obb.orientation[1] * E[1] + obb.orientation[2] * E[2];
    vertex[3] = C + obb.orientation[0] * E[0] + obb.orientation[1] * E[1] - obb.orientation[2] * E[2];
    vertex[4] = C - obb.orientation[0] * E[0] - obb.orientation[1] * E[1] - obb.orientation[2] * E[2];
    vertex[5] = C + obb.orientation[0] * E[0] - obb.orientation[1] * E[1] - obb.orientation[2] * E[2];
    vertex[6] = C - obb.orientation[0] * E[0] + obb.orientation[1] * E[1] - obb.orientation[2] * E[2];
    vertex[7] = C - obb.orientation[0] * E[0] - obb.orientation[1] * E[1] + obb.orientation[2] * E[2];
}

//Get all the edges of an OBB
void getEdgesOBB(const Collider& obb, std::vector<glm::vec3>& start_lines, std::vector<glm::vec3>& end_lines){
    start_lines.reserve(12);
    end_lines.reserve(12);

    std::vector<glm::vec3> v;
    getVerticesOBB(obb, v);

    //std::cout << "GET EDGES OBB " << std::endl;
    //print(obb.position);
    //print(obb.size);
    //print(obb.dimensions);
    //print(obb.orientation);

    //std::cout << "NB V : " << v.size() << std::endl;
    //for (size_t i = 0; i < v.size(); i++){
    //    std::cout << "POINT : " << i << std::endl;
    //    print(v[i]);
    //}


    int index[][2] = { // Indices of edges
        { 6, 1 },{ 6, 3 },{ 6, 4 },{ 2, 7 },{ 2, 5 },{ 2, 0 },
        { 0, 1 },{ 0, 3 },{ 7, 1 },{ 7, 4 },{ 4, 5 },{ 5, 3 }
    };
    for (int j = 0; j < 12; ++j) {
        start_lines.push_back(v[index[j][0]]);
        end_lines.push_back(v[index[j][1]]);
    }
}

//Get all the planes of an OBB
void getPlanesOBB(const Collider& obb, std::vector<glm::vec3>& normals_plane, std::vector<float>& distances_to_origin) {
    glm::vec3 c = obb.position;	// OBB Center
    glm::vec3 e = obb.size;		// OBB Extents

    glm::vec3 a[] = {			// OBB Axis
        obb.orientation[0],
        obb.orientation[1],
        obb.orientation[2],
    };

    normals_plane.resize(6);
    distances_to_origin.resize(6);

    normals_plane[0] = a[0];
    normals_plane[1] = a[0] * -1.0f;
    normals_plane[2] = a[1];
    normals_plane[3] = a[1] * -1.0f;
    normals_plane[4] = a[2];
    normals_plane[5] = a[2] * -1.0f;

    distances_to_origin[0] = glm::dot(a[0], (c + a[0] * e.x));
    distances_to_origin[1] = -glm::dot(a[0], (c - a[0] * e.x));
    distances_to_origin[2] = glm::dot(a[1], (c + a[1] * e.y));
    distances_to_origin[3] = -glm::dot(a[1], (c - a[1] * e.y));
    distances_to_origin[4] = glm::dot(a[2], (c + a[2] * e.z));
    distances_to_origin[5] = -glm::dot(a[2], (c - a[2] * e.z));
}

//Get all the vertices of an OBB
//Checks if a line intersects a plane and if it does, the line is clipped to the plane
bool clipToPlane(const glm::vec3& normal_plane, const float distance_to_origin_plane, const glm::vec3& start_line, const glm::vec3& end_line, glm::vec3* outPoint) {
    glm::vec3 ab = end_line - start_line;

    float nA = glm::dot(normal_plane, start_line);
    float nAB = glm::dot(normal_plane, ab);

    //No line and plane intersection
    if (compareWithEpsilon(nAB, 0.0f)) {
        return false;
    }

    //Find the time along the line at which it intersects the plane
    float t = (distance_to_origin_plane - nA) / nAB;
    if (t >= 0.0f && t <= 1.0f) {
        if (outPoint != 0) {
            *outPoint = start_line + ab * t;
        }
        return true;
    }

    //If the time is not within the range of zero to one, the plane and line segment do not intersect
    return false;
}

//The edges provided are clipped against the planes of the provided bounding box
std::vector<glm::vec3> clipEdgesToOBB(const std::vector<glm::vec3>& start_edges, const std::vector<glm::vec3>& end_edges, const Collider& obb) {
    std::vector<glm::vec3> result;
    result.reserve(start_edges.size() * 3);
    
    glm::vec3 intersection;

    std::vector<glm::vec3> normals_plane;
    std::vector<float> distances_to_origin;
    getPlanesOBB(obb, normals_plane,distances_to_origin);

    //std::cout << " CLIP TO EDGE OBB ==========="<< std::endl;
    //print(obb.position);
    //print(obb.dimensions);
    //std::cout << "NORMAL TO PLAN : " << normals_plane.size() << std::endl;
    //std::cout << "DISTANCE TO ORIGIN : " << distances_to_origin.size() << std::endl;
    //for (size_t i = 0; i < normals_plane.size(); i++) {
    //    std::cout << "PLAN I : " << i << std::endl;
    //    print(normals_plane[i]);
    //    print(distances_to_origin[i]);
    //}

    for (int i = 0; i < normals_plane.size(); ++i) {
        for (int j = 0; j < start_edges.size(); ++j) {
            if (clipToPlane(normals_plane[i], distances_to_origin[i], start_edges[j], end_edges[j], &intersection)) {
                //std::cout << "++PLANE INTERSECTION " << std::endl;
                //std::cout << "DIST TO ORIGIN : " << distances_to_origin[i] << std::endl;
                //print(start_edges[j]);
                //print(end_edges[j]);
                //print(intersection);
                if (pointInOBB(intersection, obb)) {
                    result.push_back(intersection);
                }
            }
        }
    }

    return result;
}

float penetrationDepthOBB(const Collider& o1, const Collider& o2, const glm::vec3& axis, bool* outShouldFlip) {
    glm::vec3 axisn = glm::normalize(axis);
    Interval i1 = GetIntervalOBB(o1, axisn);
    Interval i2 = GetIntervalOBB(o2, axisn);

   //std::cout << "PENETRATION DEPTH OBBOBB" << std::endl;
   //print(axis);
   //print(i1);
   //print(i2);


    if (!((i2.min <= i1.max) && (i1.min <= i2.max))) {
        return 0.0f; // No penerattion
    }

    float len1 = i1.max - i1.min;
    float len2 = i2.max - i2.min;

    float min = std::min(i1.min, i2.min);
    float max = std::max(i1.max, i2.max);
    float length = max - min;

    if (outShouldFlip != 0) {
        *outShouldFlip = (i2.min < i1.min);
    }

    return (len1 + len2) - length;
}

void OBBOBBCollision(const Collider& obb1, const Collider& obb2, ColliderResult* res) {
    //std::cout << "=============================== INTERSECTION OBB OBB" << std::endl;
    assert(obb1.type == colliderType::OBB);
    assert(obb2.type == colliderType::OBB);
    assert(res != 0);

    //SAT basic treatment
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
        test[6 + i * 3 + 0] = glm::cross(test[i], test[4]);
        test[6 + i * 3 + 1] = glm::cross(test[i], test[5]);
        test[6 + i * 3 + 2] = glm::cross(test[i], test[6]);
    }


    glm::vec3* hitNormal = 0;
    bool shouldFlip;
    float min_pen_dist = FLT_MAX;

    for (int i = 0; i < 15; ++i) {
        //No need to treat zero sized vectors
        if (glm::dot(test[i], test[i]) < 0.001f) {
            //std::cout << "ELIMINATED VECTOR" << std::endl;
            //print(test[i]);
            continue;
        }

        float depth = penetrationDepthOBB(obb1, obb2, test[i], &shouldFlip);
        if (depth <= 0.0f) {
            //We found a separation axis
            //std::cout << "SAT FOUND" << std::endl;
            //print(test[i]);
            return;
        }

        else if (depth < min_pen_dist) {
            if (shouldFlip) {
                test[i] = test[i] * -1.0f;
            }
            min_pen_dist = depth;
            hitNormal = &test[i];
        }
    }


    if (hitNormal == 0) {
        //std::cout << "                   NO NORMAL FOUND" << std::endl;
        //No axis found 
        return;
    }

    //So we got an intersection then
    res->isInCollision = true;

    
    //The axis with the minimum penetration
    //The axis with the minimum displacmeent to perform for avoiding the collision
    glm::vec3 axis = glm::normalize(*hitNormal);

    std::vector<glm::vec3> start_edges_2;
    std::vector<glm::vec3> end_edges_2;
    std::vector<glm::vec3> start_edges_1;
    std::vector<glm::vec3> end_edges_1;
    
    getEdgesOBB(obb1, start_edges_1, end_edges_1);
    getEdgesOBB(obb2, start_edges_2, end_edges_2);


    std::vector<glm::vec3> c1 = clipEdgesToOBB(start_edges_2,end_edges_2, obb1);
    std::vector<glm::vec3> c2 = clipEdgesToOBB(start_edges_1, end_edges_1, obb2);

    //std::cout << "NB C1 : " << c1.size() << std::endl;
    //std::cout << "NB C2 : " << c2.size() << std::endl;

    //All possible contact points
    std::vector<glm::vec3> pContacts;
    pContacts.reserve(c1.size() + c2.size());
    pContacts.insert(pContacts.end(), c1.begin(), c1.end());
    pContacts.insert(pContacts.end(), c2.begin(), c2.end());


    if (pContacts.size() == 0) {
        //If we dont have any point, its because one obb is in the other one
        ContactPoint* cp_res = new ContactPoint();
        cp_res->normal = axis;
        cp_res->penetrationDistance = min_pen_dist;
        //First find the box inside the other
        if (pointInOBB(obb1.position, obb2)) {
            //OBB1 in OBB2
            //Find the closest point in OBB1
            cp_res->point = closestPointOBB(obb1, obb2.position);
        }
        else {
            //OBB2 in OBB1
            cp_res->point = closestPointOBB(obb2, obb1.position);
        }
        res->contactsPts.push_back(cp_res);
    }
    else {
        //Find the OBB interval on the current chosed axis 
        Interval i = GetIntervalOBB(obb1, axis);

        float distance = (i.max - i.min) * 0.5f - min_pen_dist * 0.5f;
        glm::vec3 pointOnPlane = obb1.position + axis * distance;

        
        for (int i = pContacts.size() - 1; i >= 0; --i) {
            glm::vec3 contact = pContacts[i];
            pContacts[i] = contact + (axis * glm::dot(axis, pointOnPlane - contact));

            //Erase similar contact points
            for (int j = pContacts.size() - 1; j > i; --j) {
                glm::vec3 dif = pContacts[j] - pContacts[i];
                if (glm::dot(dif, dif) < 0.0001f) {
                    pContacts.erase(pContacts.begin() + j);
                    break;
                }
            }
        }

        if (pContacts.size() > 0) {
            for (int i = 0; i < pContacts.size(); i++) {
                ContactPoint* cp_res = new ContactPoint();
                cp_res->normal = axis;
                cp_res->point = pContacts[i];
                cp_res->penetrationDistance = min_pen_dist;

                res->contactsPts.push_back(cp_res);
            }
        }
    }
}


//Collision between an OBB and a Plane
//Plane is discribed by its normal and a distance to origin
void OBBPlaneCollision(const Collider& obb, const glm::vec3 normal_plan, float distance_to_origin, ColliderResult* res) {
    //std::cout << "INTERSECTION OBB PLANE" << std::endl;
    assert(obb.type == colliderType::OBB);
    assert(res!=0);

    ContactPoint* cp_res = 0;

    // Project the half extents of the OBB onto the plane normal
    float pLen = 
        obb.size.x * std::abs(glm::dot(normal_plan, obb.orientation[0])) +
        obb.size.y * std::abs(glm::dot(normal_plan, obb.orientation[1])) +
        obb.size.z * std::abs(glm::dot(normal_plan, obb.orientation[2]));
    // Find the distance from the center of the OBB to the plane
    float dist = glm::dot(normal_plan, obb.position) - distance_to_origin;
    // Intersection occurs if the distance falls within the projected side
    if (std::abs(dist) <= pLen) {
        cp_res = new ContactPoint();

        cp_res->penetrationDistance = pLen - std::abs(dist);
        cp_res->normal = (dist < 0) ? -normal_plan : normal_plan;
        cp_res->point = obb.position + (cp_res->normal  * (std::abs(dist) - cp_res->penetrationDistance));
    }

    //If there is a contact point
    if (cp_res != 0) {
        res->isInCollision = true;
        res->contactsPts.push_back(cp_res);
    }
}


//Intersection function
void intersect(Collider c1, Collider c2, ColliderResult* res) {
    //Compute collisions depend on the collider type
    if (isSupportedCollider(c1) && isSupportedCollider(c2)) {
        if (c1.type == colliderType::AABB && c2.type == colliderType::AABB) {
            //std::cout << "INTERSECT AABB AABB" << std::endl;
            AABBAABBCollision(c1, c2, res);
            //if (res != 0) res->inverseNormals();
        }
        if (c1.type == colliderType::AABB && c2.type == colliderType::Sphere) {
            //std::cout << "INTERSECT AABB SPHERE" << std::endl;
            SphereAABBCollision(c1, c2, res);
            if (res != 0) res->inverseNormals();
        }
        if (c1.type == colliderType::Sphere && c2.type == colliderType::AABB) {
            //std::cout << "INTERSECT SPHERE AABB" << std::endl;
            SphereAABBCollision(c2, c1, res);
            //if (res != 0) res->inverseNormals();
        }
        if (c1.type == colliderType::OBB && c2.type == colliderType::AABB) {
            //std::cout << "INTERSECT OBB AABB" << std::endl;
            AABBOBBCollision(c2, c1, res);
            //if (res != 0) res->inverseNormals();
        }
        if (c1.type == colliderType::AABB && c2.type == colliderType::OBB) {
            //std::cout << "INTERSECT AABB OBB" << std::endl;
            AABBOBBCollision(c1, c2,res);
            if (res != 0) res->inverseNormals();
        }
        if (c1.type == colliderType::Sphere && c2.type == colliderType::Sphere) {
            //std::cout << "INTERSECT SPHERE SPHERE" << std::endl;
            SphereSphereCollision(c1, c2,res);
        }
        if (c1.type == colliderType::Sphere && c2.type == colliderType::OBB) {
            //std::cout << "INTERSECT SPHERE OBB" << std::endl;
            OBBSphereCollision(c1, c2,res);
            //if (res != 0) res->inverseNormals();
        }
        if (c1.type == colliderType::OBB && c2.type == colliderType::Sphere) {
            //std::cout << "INTERSECT OBB SPHERE" << std::endl;
            OBBSphereCollision(c2, c1, res);
            if (res != 0) res->inverseNormals();
        }
        if (c1.type == colliderType::OBB && c2.type == colliderType::OBB) {
            //std::cout << "INTERSECT OBB OBB" << std::endl;
            OBBOBBCollision(c1, c2, res);
            if (res != 0) res->inverseNormals();
        }

        //if(res != 0 && res->isInCollision) std::cout << "INTERSECTION !" << std::endl;
        //else std::cout << "NO INTERSECTION !" << std::endl;
    }
}

std::vector<RaycastResult*> ColliderSystem::rayCastAll(const Ray& ray) {
    std::vector<RaycastResult*> results;
    results.reserve(entityIDs.size());
    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        if (!EntityManager::instance->shouldUpdate(entityIDs[i]))
            continue;

        Collider* c = getCollider(i);
        RaycastResult* result = new RaycastResult();
        if (RayCastCollider(*c, ray, result)) {
            results.push_back(result);
        }
    }
    return results;
}

/*============================ JUST COLLIDER INTERSECTION : NO NORMAL AND POSITION CALC ============================*/
bool OverlapOnAxisAABBOBB(const Collider& aabb, const Collider& obb, const glm::vec3& axis) {
    assert(aabb.type == colliderType::AABB);
    assert(obb.type == colliderType::OBB);
    ColliderResult* res = new ColliderResult();
    Interval a = GetIntervalAABB(aabb, axis);
    Interval b = GetIntervalOBB(obb, axis);
    return ((b.min <= a.max) && (a.min <= b.max));
 }
bool OverlapOnAxisOBBOBB(const Collider& obb1, const Collider& obb2, const glm::vec3& axis) {
    assert(obb1.type == colliderType::OBB);
    assert(obb2.type == colliderType::OBB);

    ColliderResult* res = new ColliderResult();
    Interval a = GetIntervalOBB(obb1, axis);
    Interval b = GetIntervalOBB(obb2, axis);
    return ((b.min <= a.max) && (a.min <= b.max));
}

bool isAABBAABBCollision(const Collider& aabb1, const Collider& aabb2) {
    //std::cout << "IS IN INTERSECTION AABB AABB" << std::endl;
    assert(aabb1.type == colliderType::AABB);
    assert(aabb2.type == colliderType::AABB);

    glm::vec3 min_1;
    glm::vec3 min_2;
    glm::vec3 max_1;
    glm::vec3 max_2;
    computeMinMaxAABB(aabb1, min_1, max_1);
    computeMinMaxAABB(aabb2, min_2, max_2);

    bool overlap_x = ((aabb1.position.x + min_1.x) < (aabb2.position.x + max_2.x)) && ((aabb2.position.x + min_2.x) < (aabb1.position.x + max_1.x));
    bool overlap_y = ((aabb1.position.y + min_1.y) < (aabb2.position.y + max_2.y)) && ((aabb2.position.y + min_2.y) < (aabb1.position.y + max_1.y));
    bool overlap_z = ((aabb1.position.z + min_1.z) < (aabb2.position.z + max_2.z)) && ((aabb2.position.z + min_2.z) < (aabb1.position.z + max_1.z));
    return overlap_x && overlap_y && overlap_z;
}
bool isSphereSphereCollision(Collider sp1, Collider sp2) {
    //std::cout << "IS IN INTERSECTION SPHERE SPHERE" << std::endl;
    assert(sp1.type == colliderType::Sphere);
    assert(sp2.type == colliderType::Sphere);
    float r = sp1.radius + sp2.radius;
    glm::vec3 mvt = sp1.position - sp2.position;
    float distance = glm::length(mvt);
    return distance < r;
}
bool isSphereAABBCollision(const Collider& aabb, const Collider& sphere) {
    //std::cout << "IS IN INTERSECTION AABB SPHERE" << std::endl;
    assert(aabb.type == colliderType::AABB);
    assert(sphere.type == colliderType::Sphere);

    glm::vec3 min;
    glm::vec3 max;
    computeMinMaxAABB(aabb, min, max);

    glm::vec3 difference = sphere.position - aabb.position;

    //Get the coord of the point in the sphere at smalest dist from the cube
    float x = std::max(min.x, std::min(difference.x, max.x));
    float y = std::max(min.y, std::min(difference.y, max.y));
    float z = std::max(min.z, std::min(difference.z, max.z));

    difference = (glm::vec3(x, y, z) + aabb.position) - sphere.position;
    float distance = glm::length(difference);

    return distance < sphere.radius;
}
bool isSphereOBBCollision(const Collider& sphere, const Collider& obb) {
    //std::cout << "IS IN INTERSECTION SPHERE OBB" << std::endl;
    assert(sphere.type == colliderType::Sphere);
    assert(obb.type == colliderType::OBB);
    glm::vec3 closestPoint = closestPointOBB(obb, sphere.position);
    glm::vec3 dif = sphere.position - closestPoint;
    float dist_squared = glm::dot(dif, dif);
    float radius_squared = sphere.radius * sphere.radius;
    return (dist_squared < radius_squared);
}
bool isSpherePlaneCollision(const Collider& sphere, const glm::vec3 normal_plan, float distance_to_origin) {
    //std::cout << "IS IN INTERSECTION SPHERE PLANE" << std::endl;
    assert(sphere.type == colliderType::Sphere);
    glm::vec3 closestPoint = closestPointPlane(sphere.position, normal_plan, distance_to_origin);
    glm::vec3 dif = sphere.position - closestPoint;
    float dist_squared = glm::dot(dif, dif);
    float radius_squared = sphere.radius * sphere.radius;
    return (dist_squared < radius_squared);
}
bool isAABBOBBCollision(const Collider& aabb, const Collider& obb) {
    //std::cout << "IS IN INTERSECTION AABB OBB" << std::endl;
    assert(aabb.type == colliderType::AABB);
    assert(obb.type == colliderType::OBB);
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
        test[6 + i * 3 + 0] = glm::cross(test[i], test[3]);
        test[6 + i * 3 + 1] = glm::cross(test[i], test[4]);
        test[6 + i * 3 + 2] = glm::cross(test[i], test[5]);
    }

    for (int i = 0; i < 15; ++i) {
        if (glm::dot(test[i], test[i]) < 0.001f) {
            continue;
        }
        if (!OverlapOnAxisAABBOBB(aabb, obb, test[i])) {
            // Seperating axis found
            return false;
        }
    }
    return true;
}
bool isAABBPlaneCollision(const Collider& aabb, const glm::vec3 normal_plan, float distance_to_origin) {
    //std::cout << "IS IN INTERSECTION AABB PLANE" << std::endl;
    assert(aabb.type == colliderType::AABB);
    // Project the half extents of the AABB onto the plane normal
    float pLen =
        aabb.size.x * std::abs(normal_plan.x) +
        aabb.size.y * std::abs(normal_plan.y) +
        aabb.size.z * std::abs(normal_plan.z);
    // Find the distance from the center of the AABB to the plane
    float dist = glm::dot(normal_plan, aabb.position) - distance_to_origin;
    // Intersection occurs if the distance falls within the projected side
    return (std::abs(dist) <= pLen);
}
bool isOBBOBBCollision(const Collider& obb1, const Collider& obb2) {
    //std::cout << "IS IN INTERSECTION OBB OBB" << std::endl;
    assert(obb1.type == colliderType::OBB);
    assert(obb2.type == colliderType::OBB);

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
        test[6 + i * 3 + 0] = glm::cross(test[i], test[4]);
        test[6 + i * 3 + 1] = glm::cross(test[i], test[5]);
        test[6 + i * 3 + 2] = glm::cross(test[i], test[6]);
    }

    for (int i = 0; i < 15; ++i) {
        if (glm::dot(test[i], test[i]) < 0.001f) {
            continue;
        }
        if (!OverlapOnAxisOBBOBB(obb1, obb2, test[i])) {
            return false;
        }
    }
    return true;
}
bool isOBBPlaneCollision(const Collider& obb, const glm::vec3 normal_plan, float distance_to_origin) {
    //std::cout << "IS IN INTERSECTION OBB PLANE" << std::endl;
    assert(obb.type == colliderType::OBB);

    // Project the half extents of the AABB onto the plane normal
    float pLen =
        obb.size.x * std::abs(glm::dot(normal_plan, obb.orientation[0])) +
        obb.size.y * std::abs(glm::dot(normal_plan, obb.orientation[1])) +
        obb.size.z * std::abs(glm::dot(normal_plan, obb.orientation[2]));
    // Find the distance from the center of the OBB to the plane
    float dist = glm::dot(normal_plan, obb.position) - distance_to_origin;
    // Intersection occurs if the distance falls within the projected side
    return (std::abs(dist) <= pLen);
}

//Collision between two Planes
//Planes are discribed by their normals and a distance to origin
bool PlanePlaneIntersection(const glm::vec3 normal_plan_1, float distance_to_origin_1, const glm::vec3 normal_plan_2, float distance_to_origin_2) {
    // << "IS IN INTERSECTION 3 PLANES" << std::endl;
    // Compute direction of intersection line
    glm::vec3 d = glm::cross(normal_plan_1, normal_plan_2);
    // Check the length of the direction line
    // if the length is 0, no intersection happened
    return !(compareWithEpsilon(glm::dot(d, d), 0));
}

bool isInIntersection(Collider c1, Collider c2) {
    if (c1.type == colliderType::AABB && c2.type == colliderType::AABB)     return isAABBAABBCollision(c1, c2);
    if (c1.type == colliderType::AABB && c2.type == colliderType::Sphere)   return isSphereAABBCollision(c1, c2);
    if (c1.type == colliderType::Sphere && c2.type == colliderType::AABB)   return isSphereAABBCollision(c2, c1);
    if (c1.type == colliderType::OBB && c2.type == colliderType::AABB)      return isAABBOBBCollision(c2, c1);
    if (c1.type == colliderType::AABB && c2.type == colliderType::OBB)      return isAABBOBBCollision(c1, c2);
    if (c1.type == colliderType::Sphere && c2.type == colliderType::Sphere) return isSphereSphereCollision(c1, c2);
    if (c1.type == colliderType::Sphere && c2.type == colliderType::OBB)    return isSphereOBBCollision(c1, c2);
    if (c1.type == colliderType::OBB && c2.type == colliderType::Sphere)    return isSphereOBBCollision(c2, c1);
    if (c1.type == colliderType::OBB && c2.type == colliderType::OBB)       return isOBBOBBCollision(c2, c1);
    
    std::cout << "NO COLLISION TEST FOUND !" << std::endl;
    return false;
}


void ColliderSystem::computeIntersection(unsigned short i, unsigned short entityIID, unsigned short j, unsigned short entityJID){
    Collider* c_i = getCollider(i);
    Collider* c_j = getCollider(j);

    ColliderResult* res = collisionResultMap.at(entityIID)[j];
    if (res == 0) {
        res = new ColliderResult();
    }
    else {
        res->contactsPts.clear();
        res->isInCollision = false;
    }

    //std::cout << " ============= COMPUTE INTERSECTION ============= "<< std::endl;
    //std::cout << " -> COMPUTE INTERSECTION IN : " << entityIID << std::endl;
    //std::cout << " -> COMPUTE INTERSECTION OUT : " << entityJID << std::endl;
    
    //If the entity have a rb, we compute a more precise collision
    if (EntityManager::instance->hasComponent(SystemIDs::RigidBodyID, entityIID) && EntityManager::instance->hasComponent(SystemIDs::RigidBodyID, entityJID)) {
        
        intersect(*c_i, *c_j,res);

        //std::cout << "END OF INTERSECTION" << std::endl;

        if(res) res->entityCollidID = entityJID;
        collisionResultMap.at(entityIID)[j] = res;

        
        if (collisionResultMap.at(entityJID)[i] == 0) {
            ColliderResult* cr = new ColliderResult(entityIID, res, false);
            cr->contactsPts.clear();
            collisionResultMap.at(entityJID)[i] = cr;
        }
        else {
            collisionResultMap.at(entityJID)[i]->isInCollision = res->isInCollision;
            collisionResultMap.at(entityJID)[i]->contactsPts.clear();
        }
    }
    else { //we just compute intersection answer

        res->isInCollision = isInIntersection(*c_i, *c_j);
        res->entityCollidID = entityJID;

        collisionResultMap.at(entityIID)[j] = res;
        
        if (collisionResultMap.at(entityJID)[i] == 0) {
            ColliderResult* cr = new ColliderResult(entityIID, res, false);
            collisionResultMap.at(entityJID)[i] = cr;
        }
        else {
            collisionResultMap.at(entityJID)[i]->contactsPts.clear();
            collisionResultMap.at(entityJID)[i]->isInCollision = res->isInCollision;
        }
        
        //std::cout << "END OF IS IN INTERSECTION" << std::endl;

        //simpleCollisionResultMap.at(entityIID)[j] = answer;
        //simpleCollisionResultMap.at(entityJID)[i] = answer;
    }
}


std::vector<ColliderResult*> ColliderSystem::getResultOf(unsigned int entityID) {
    std::map<unsigned short, std::vector<ColliderResult*>>::iterator it = collisionResultMap.find(entityID);
    if (it == collisionResultMap.end())
        return std::vector<ColliderResult*>();
    return it->second;
}