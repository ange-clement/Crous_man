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

ColliderResult::ColliderResult() {
    this->penetrationDistance = FLT_MAX;
    this->isInCollision = false;
    this->normal =glm::vec3(0,0,1);
    this->point = glm::vec3(0);
}

ColliderResult::ColliderResult(unsigned short id, ColliderResult* c) {
    this->entityCollidID = id;
    this->isInCollision = c->isInCollision;
    this->penetrationDistance = c->penetrationDistance;
    this->point = c->point;
    this->normal = c->normal;
}

ColliderResult::~ColliderResult() {

}


void ColliderResult::print() {
    std::cout << this->isInCollision << "," << this->penetrationDistance << "p : {" << this->point.x << "," << this->point.y << "," << this->point.z << "}" << "n : {" << this->normal.x << "," << this->normal.y << "," << this->normal.z << "}" << std::endl;
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
    getCollider(i)->entityID    = entityID;

    if (getCollider(i)->type == colliderType::Sphere) {
        getCollider(i)->dimensions = glm::vec3(getCollider(i)->radius);
    }
    if (getCollider(i)->type == colliderType::OBB) {
        getCollider(i)->dimensions = getCollider(i)->size;
     }
    if (getCollider(i)->type == colliderType::AABB) {
        getCollider(i)->orientation = glm::mat3(1);
    }

    //Init collider datas
    std::vector<ColliderResult*> res;
    res.resize(entityIDs.size(), 0);
    collisionResultMap.insert(CollisionResultMap::value_type(entityID, res));
}

void ColliderSystem::update(unsigned short i, unsigned short entityID) {
    //We move all the positions from all colliders
    Entity* entity = EntityManager::instance->entities[entityID];
    getCollider(i)->position = entity->worldTransform->translation + getCollider(i)->center;

    //We also need to perform others implementations like dimensions for AABB
    if (getCollider(i)->type == colliderType::AABB) {
        computeAABBDimensions(getCollider(i));
    }

    if (getCollider(i)->type == colliderType::OBB) {
        getCollider(i)->orientation = EntityManager::instance->entities[entityID]->worldTransform->rotation.rotationMatrix;
    }

    if (glfwGetKey(InputManager::instance->window, GLFW_KEY_C) == GLFW_PRESS) {
        drawCollider(i);
    }
}
void ColliderSystem::drawCollider(unsigned short i) {
    Collider* c = getCollider(i);
    c->draw = true;
}

void ColliderSystem::addEntityComponent() {
    EntityManager::instance->colliderComponents.push_back(Collider());
}

void ColliderSystem::renderAll(glm::mat4 view, glm::mat4 projection) {
    unsigned short entityID;
    for (size_t i = 0, size = entityIDs.size(); i < size; i++) {
        entityID = entityIDs[i];

        if (entityID == (unsigned short)-1) {
            continue;
        }

        Collider* c = getCollider(i);
        if (!c->drawable || !c->draw) {
            continue;
        }
        Entity* e = EntityManager::instance->entities[entityID];
        c->shader->use();

        //std::cout << "RENDER ALL" << std::endl;
        //print(c->orientation);
        //print(c->dimensions);

        glm::mat4 model = e->worldTransform->toMat4NoScalingNoRotation();
        c->shader->setMVPC(model * glm::mat4(c->orientation), view, projection, isInContactWithSomething(entityID), c->dimensions);
        c->shader->draw();
    }
}

Collider* ColliderSystem::getCollider(unsigned short i) {
    return &EntityManager::instance->colliderComponents[i];
}

Collider* ColliderSystem::getColliderEntityID(unsigned short entityID) {
    for (size_t j = 0, size = entityIDs.size(); j < size; j++) {
        Collider* c = getCollider(j);
        if (c->entityID == entityID) return c;
    }
    return 0;
}


/* =============== Methods for collision detection and resolution =============== */
void ColliderSystem::updateCollision(unsigned short i, unsigned short entityID) {
    unsigned short entityIDJ;
    Collider* c_i = getCollider(i);

    std::vector<ColliderResult*> res = collisionResultMap.at(entityID);


    for (size_t j = i+1, size = entityIDs.size(); j < size; j++) {
        //if (!res[j]) {
            entityIDJ = entityIDs[j];
            Collider* c_j = getCollider(j);

            ColliderResult* resC = intersect(*c_i, *c_j);
            resC->entityCollidID = entityIDJ;
            //std::cout << "result of collision : " << ((resC->isInCollision) ? "TRUE" : "FALSE") << " => (" << i << "," << j << ")" << std::endl;
            collisionResultMap.at(entityID)[j] = resC;
            collisionResultMap.at(entityIDJ)[i] = new ColliderResult(entityID,resC);
        //}
    }
    //std::cout << " === END UPDATE COLLISION === " << std::endl;
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

bool ColliderSystem::isInContactWithSomething(unsigned short i) {
    //std::cout << "IS IN CONTACT WITH SMT : " << i << std::endl;
    for (size_t j = 0; j < entityIDs.size(); j++){
        ColliderResult* r = collisionResultMap.at(i)[j];

        if (r) {
            if(r->isInCollision) return true;
            //std::cout << "IIC no : " << i << "," << j << std::endl;
        }
    }
    return false;
}

void logCollisionResultMap(CollisionResultMap m) {
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

void ColliderSystem::clearAllCollision(unsigned short i) {
    for (size_t j = 0, size = entityIDs.size(); j < size; j++) {
        collisionResultMap.at(i)[j] = 0;
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
            ColliderResult* res = intersect(*c_first,*c_second);
            collisionResultMap[i][j] = res;
            collisionResultMap[j][i] = res;
        }
    }
}
void ColliderSystem::QuadTreeCollisionResolution() {

}
void ColliderSystem::OcTreeCollisionResolution() {

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
    min = -aabb.dimensions;
    max = aabb.dimensions;
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
    min += aabb.position;
    max += aabb.position;

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
ColliderResult* AABBAABBCollision(const Collider& aabb1, const Collider& aabb2) {
    assert(aabb1.type == colliderType::AABB);
    assert(aabb2.type == colliderType::AABB);

    ColliderResult* res = new ColliderResult();
   
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
        res->isInCollision = true;
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
            res->penetrationDistance = x;
            res->normal = glm::vec3(x_factor,0,0);
        }
        else if (y <= x && y <= z) {
            res->penetrationDistance = y;
            res->normal = glm::vec3(0, y_factor, 0);
        }
        else if (z <= y && z <= x) {
            res->penetrationDistance = z;
            res->normal = glm::vec3(0, 0, z_factor);
        }

        //To compute the point, we chose the mid point for the two projections point on both AABB
        res->point.x += ((x_factor < 0) ? max_1.x - x : min_1.x + x);
        res->point.y += ((y_factor < 0) ? max_1.y - y : min_1.y + y);
        res->point.z += ((z_factor < 0) ? max_1.z - z : min_1.z + z);
    }
    return res;
}

//Collision between two Sphere
ColliderResult* SphereSphereCollision(Collider sp1, Collider sp2) {
    assert(sp1.type == colliderType::Sphere);
    assert(sp2.type == colliderType::Sphere);

    ColliderResult* res = new ColliderResult();
   
    float r = sp1.radius + sp2.radius;


    glm::vec3 mvt = sp1.position - sp2.position;

    float distance = glm::length(mvt);
    if (distance < r) {
        res->isInCollision = true;
        //res->penetrationDistance = (sp1.radius - distance) + sp2.radius;
        res->penetrationDistance = (distance - r) *0.5f;
        res->normal = ((distance == 0) ? glm::vec3(0, 1, 0) : glm::normalize(mvt));

        float distToInterP = sp1.radius - res->penetrationDistance;
        res->point = sp1.position + (res->normal * distToInterP);
    }

    return res;
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
            res->point = sphere.position;


            if (dist.y <= dist.x && dist.y <= dist.z) {
                res->penetrationDistance = (dist.y + sphere.radius) * .5;
                res->normal = glm::vec3(0, 1, 0);
            }
            else if (dist.x <= dist.y && dist.x <= dist.z) {
                res->penetrationDistance = (dist.x + sphere.radius) * .5;
                res->normal = glm::vec3(1, 0, 0);
            }
            else if (dist.z <= dist.y && dist.z <= dist.x) {
                res->penetrationDistance = (dist.z + sphere.radius)* .5;
                res->normal = glm::vec3(0, 0, 1);
            }
        }
        else {
            //We need to know the direction of the minimum penetration
            res->normal = perform_direction(difference, res->penetrationDistance, sphere.radius);
            //We also need to know point to apply normal and distance
        }
    }
    return res;
}

ColliderResult* SphereAABBCollision(const Collider& aabb, const Collider& sphere) {
    assert(aabb.type == colliderType::AABB);
    assert(sphere.type == colliderType::Sphere);

    ColliderResult* res = new ColliderResult();

    glm::vec3 closestPoint = closestPointAABB(aabb, sphere.position);

    glm::vec3 dif = sphere.position - closestPoint;
    float dist_squared = glm::dot(dif,dif);
    float radius_squared = sphere.radius * sphere.radius;

    if (dist_squared < radius_squared) {
        //Collision !
        //We got a collision then
        glm::vec3 normal;

        //closest point correspond to sphere center
        if (compareWithEpsilon(dist_squared, 0.0f)) {
            glm::vec3 dif_aabb = closestPoint - aabb.position;
            
            if (compareWithEpsilon(glm::dot(dif_aabb, dif_aabb), 0.0f)) {
                //Both are on the same place, we can then propulse from one of the center trougth any direction with penetration of rad+mindim/2
                res->isInCollision = true;
                res->point = aabb.position;

                glm::vec3 dist = aabb.dimensions * 2.0f;

                if (dist.y <= dist.x && dist.y <= dist.z) {
                    res->penetrationDistance = (dist.y + sphere.radius) * .5;
                    res->normal = glm::vec3(0, 1, 0);
                }
                else if (dist.x <= dist.y && dist.x <= dist.z) {
                    res->penetrationDistance = (dist.x + sphere.radius) * .5;
                    res->normal = glm::vec3(1, 0, 0);
                }
                else if (dist.z <= dist.y && dist.z <= dist.x) {
                    res->penetrationDistance = (dist.z + sphere.radius) * .5;
                    res->normal = glm::vec3(0, 0, 1);
                }
                
                return res;
            }

            // Closest point is at the center of the sphere
            normal = glm::normalize(dif_aabb);
        }
        else {
            normal = glm::normalize(dif);
        }

        glm::vec3 outsidePoint = sphere.position - normal * sphere.radius;

        float distance = glm::length(closestPoint - outsidePoint);

        res->isInCollision = true;
        res->point = closestPoint + (outsidePoint - closestPoint) * 0.5f;
        res->normal = normal;
        res->penetrationDistance = distance * 0.5f;
    }

    return res;
}


//Collision between an OBB and a sphere
ColliderResult* SphereOBBCollision(const Collider& sphere, const Collider& obb) {
    assert(sphere.type == colliderType::Sphere);
    assert(obb.type == colliderType::OBB);

    ColliderResult* res = new ColliderResult();

    glm::vec3 closestPoint = closestPointOBB(obb, sphere.position);
    
    glm::vec3 dif = sphere.position - closestPoint;

    float dist_squared = glm::dot(dif,dif);
    float radius_squared = sphere.radius * sphere.radius;

    if (dist_squared < radius_squared) {
        //We got a collision then
        glm::vec3 normal;

        if (compareWithEpsilon(dist_squared, 0.0f)) {
            glm::vec3 dif_obb = closestPoint - obb.position;
            if (compareWithEpsilon(glm::dot(dif_obb, dif_obb), 0.0f)) {
                //Both are in the same area, we can move the Sphere on the smalest dimension of the OBB
                res->point = sphere.center;
                
                Interval i_x =  GetIntervalOBB(obb, glm::vec3(1,0,0));
                Interval i_y =  GetIntervalOBB(obb, glm::vec3(0,1,0));
                Interval i_z =  GetIntervalOBB(obb, glm::vec3(0,0,1));
                
                glm::vec3 dist = glm::vec3(i_x.max - i_x.min, i_y.max - i_y.min, i_z.max - i_z.min);
                if (dist.y <= dist.x && dist.y <= dist.z) {
                    res->penetrationDistance = (dist.y + sphere.radius) * .5;
                    res->normal = obb.orientation[1];
                }
                else if (dist.x <= dist.y && dist.x <= dist.z) {
                    res->penetrationDistance = (dist.x + sphere.radius) * .5;
                    res->normal = obb.orientation[0];
                }
                else if (dist.z <= dist.y && dist.z <= dist.x) {
                    res->penetrationDistance = (dist.z + sphere.radius) * .5;
                    res->normal = obb.orientation[2];
                }
                return res;
            }

            // Closest point is at the center of the sphere
            normal = glm::normalize(dif_obb);
        }else {
            normal = glm::normalize(dif);
        }

        glm::vec3 outsidePoint = sphere.position - normal * sphere.radius;

        float distance = glm::length(closestPoint - outsidePoint);

        res->isInCollision = true;
        res->point = closestPoint + (outsidePoint - closestPoint) * 0.5f;
        res->normal = normal;
        res->penetrationDistance = distance * 0.5f;
    }

    return res;
}

//Collision between a plane and a sphere
//Plane is discribed by its normal and a distance to origin
//TODO TEST
ColliderResult* SpherePlaneCollision(const Collider& sphere, const glm::vec3 normal_plan, float distance_to_origin) {
    assert(sphere.type == colliderType::Sphere);

    ColliderResult* res = new ColliderResult();

    glm::vec3 closestPoint = closestPointPlane(sphere.position, normal_plan,distance_to_origin);
    glm::vec3 dif = sphere.position - closestPoint;
    float dist_squared = glm::dot(dif, dif);
    float radius_squared = sphere.radius * sphere.radius;

    if (dist_squared < radius_squared) {
        //We got a collision then       
        res->normal = glm::normalize(dif);
        res->point = closestPoint;
        res->penetrationDistance = (glm::length(dif) - sphere.radius) * 0.5;
        res->isInCollision = true;
    }
    return res;
}

//Collision between an AABB and an OBB
/*ColliderResult* AABBOBBCollision(const Collider& aabb, const Collider& obb) {
    assert(aabb.type == colliderType::AABB);
    assert(obb.type == colliderType::OBB);

    ColliderResult* res = new ColliderResult();

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
        //std::cout << "=========== SEEING THIS AXIS : ";
        //print(test[i]);

        ColliderResult* cres = OverlapOnAxisAABBOBB(aabb, obb, test[i]);
        if (!cres->isInCollision) {
            // Seperating axis found
            //std::cout << "SEPARATING AXIS FOUND" << std::endl;
            //print(test[i]);

            delete cres;
            return res;
        }
        else {
            //std::cout << "NOT A SEPARATING AXIS" << std::endl;
            //print(test[i]);
            //else if this axe is less distance to answer collision
            if (cres->penetrationDistance < res->penetrationDistance) {
                res->normal = test[i];
                res->penetrationDistance = cres->penetrationDistance;
            }
        }
        delete cres;
    }
    //std::cout << "===== AABBOBB COL FIN" << std::endl;
    // Seperating axis not found
    //We have an intersection here
    res->isInCollision = true;
    return res;
}*/

void getVerticesAABB(const Collider& aabb, std::vector<glm::vec3>& vertex) {
    vertex.resize(8);

    vertex.push_back(glm::vec3(-1, -1, 1) * aabb.dimensions);
    vertex.push_back(glm::vec3(1, -1, 1) *aabb.dimensions);
    vertex.push_back(glm::vec3(-1, 1, 1) * aabb.dimensions);
    vertex.push_back(glm::vec3(1, 1, 1) * aabb.dimensions);
    vertex.push_back(glm::vec3(-1, -1, -1) * aabb.dimensions);
    vertex.push_back(glm::vec3(1, -1, -1) * aabb.dimensions);
    vertex.push_back(glm::vec3(-1, 1, -1) * aabb.dimensions);
    vertex.push_back(glm::vec3(1, 1, -1) * aabb.dimensions);
}

void getEdgesAABB(const Collider& aabb, std::vector<glm::vec3>& start_lines, std::vector<glm::vec3>& end_lines) {
    start_lines.reserve(12);
    end_lines.reserve(12);

    std::vector<glm::vec3> v;
    getVerticesOBB(aabb, v);

    int index[][2] = { // Indices of edges
        { 0, 1 },{ 2, 3 },{ 0, 2 },{ 1, 3 },{ 3, 7 },{ 1, 5 },
        { 4, 5 },{ 4, 6 },{ 6, 7 },{ 7, 5 },{ 6, 2 },{ 4, 0 }
    };
    for (int j = 0; j < 12; ++j) {
        start_lines.push_back(v[index[j][0]]);
        end_lines.push_back(v[index[j][1]]);
    }
}

void getPlanesAABB(const Collider& aabb, std::vector<glm::vec3>& normals_plane, std::vector<float>& distances_to_origin) {
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

std::vector<glm::vec3> clipEdgesToAABB(const std::vector<glm::vec3>& start_edges, const std::vector<glm::vec3>& end_edges, const Collider& aabb) {
    std::vector<glm::vec3> result;
    result.reserve(start_edges.size() * 3);

    glm::vec3 intersection;

    std::vector<glm::vec3> normals_plane;
    std::vector<float> distances_to_origin;
    getPlanesAABB(aabb, normals_plane, distances_to_origin);

    for (int i = 0; i < normals_plane.size(); ++i) {
        for (int j = 0; j < start_edges.size(); ++j) {
            if (clipToPlane(normals_plane[i], distances_to_origin[i], start_edges[j], end_edges[j], &intersection)) {
                if (pointInAABB(intersection, aabb)) {
                    result.push_back(intersection);
                }
            }
        }
    }

    return result;
}

float penetrationDepthAABBOBB(const Collider& aabb, const Collider& obb, const glm::vec3& axis, bool* outShouldFlip) {
    glm::vec3 axisn = glm::normalize(axis);
    Interval i1 = GetIntervalOBB(aabb, axisn);
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


ColliderResult* AABBOBBCollision(const Collider& aabb, const Collider& obb) {
    assert(aabb.type == colliderType::AABB);
    assert(obb.type == colliderType::OBB);

    ColliderResult* res = new ColliderResult();

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

    for (int i = 0; i < 15; ++i) {
        //No need to treat zero sized vectors
        if (glm::dot(test[i], test[i]) < 0.001f) {
            continue;
        }

        float depth = penetrationDepthAABBOBB(aabb, obb, test[i], &shouldFlip);
        if (depth <= 0.0f) {
            //We found a separation axis
            return res;
        }

        else if (depth < res->penetrationDistance) {
            if (shouldFlip) {
                test[i] = test[i] * -1.0f;
            }
            res->penetrationDistance = depth;
            hitNormal = &test[i];
        }
    }


    if (hitNormal == 0) {
        //No axis found 
        return res;
    }
    glm::vec3 axis = glm::normalize(*hitNormal);

    std::vector<glm::vec3> start_edges_2;
    std::vector<glm::vec3> end_edges_2;

    std::vector<glm::vec3> start_edges_1;
    std::vector<glm::vec3> end_edges_1;

    getEdgesAABB(aabb, start_edges_1, end_edges_1);
    getEdgesOBB(obb, start_edges_2, end_edges_2);

    std::vector<glm::vec3> c1 = clipEdgesToAABB(start_edges_2, end_edges_2, aabb);
    std::vector<glm::vec3> c2 = clipEdgesToOBB(start_edges_1, end_edges_1, obb);

    std::vector<glm::vec3> pContacts;

    pContacts.reserve(c1.size() + c2.size());
    pContacts.insert(pContacts.end(), c1.begin(), c1.end());
    pContacts.insert(pContacts.end(), c2.begin(), c2.end());

    Interval i = GetIntervalOBB(obb, axis);

    float distance = (i.max - i.min) * 0.5f - res->penetrationDistance * 0.5f;
    glm::vec3 pointOnPlane = obb.position + axis * distance;

    for (int i = pContacts.size() - 1; i >= 0; --i) {
        glm::vec3 contact = pContacts[i];
        pContacts[i] = contact + (axis * glm::dot(axis, pointOnPlane - contact));

        // This bit is in the "There is more" section of the book
        for (int j = pContacts.size() - 1; j > i; --j) {
            glm::vec3 dif = pContacts[j] - pContacts[i];
            if (glm::dot(dif, dif) < 0.0001f) {
                pContacts.erase(pContacts.begin() + j);
                break;
            }
        }
    }

    res->isInCollision = true;
    res->normal = axis;
    return res;
}

//Collision between an AABB and a Plane
//Plane is discribed by its normal and a distance to origin
//TODO TEST
ColliderResult* AABBPlaneCollision(const Collider& aabb, const glm::vec3 normal_plan, float distance_to_origin) {
    assert(aabb.type == colliderType::AABB);

    ColliderResult* res = new ColliderResult();
    
    // Project the half extents of the AABB onto the plane normal
    float pLen = 
        aabb.size.x * std::abs(normal_plan.x) +
        aabb.size.y * std::abs(normal_plan.y) +
        aabb.size.z * std::abs(normal_plan.z);

    // Find the distance from the center of the AABB to the plane
    float dist = glm::dot(normal_plan, aabb.position) - distance_to_origin;
    // Intersection occurs if the distance falls within the projected side
    if (std::abs(dist) <= pLen) {
        res->isInCollision = true;
        res->penetrationDistance = pLen - std::abs(dist);
        res->normal = (dist<0)? -normal_plan : normal_plan;
    }
    return res;
}

//Collision between two OBB
/*
ColliderResult* OBBOBBCollision(const Collider& obb1, const Collider& obb2) {
    assert(obb1.type == colliderType::OBB);
    assert(obb2.type == colliderType::OBB);

    //std::cout << "====== OBBOBB COL ======" << std::endl;
    //std::cout << "obb1 position" << std::endl;
    //print(obb1.position);
    //std::cout << "obb1 size" << std::endl;
    //print(obb1.size);
    //std::cout << "obb1 dimensions" << std::endl;
    //print(obb1.dimensions);

    //std::cout << "obb2 position" << std::endl;
    //print(obb2.position);
    //std::cout << "obb2 size" << std::endl;
    //print(obb2.size);
    //std::cout << "obb2 dimensions" << std::endl;
    //print(obb2.dimensions);
    

    ColliderResult* res = new ColliderResult();

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
        //std::cout << "=========== SEEING THIS AXIS : ";
        //print(test[i]);
        ColliderResult* cres = OverlapOnAxisOBBOBB(obb1, obb2, test[i]);
        if (!cres->isInCollision) {
            //std::cout << "SEPARATING AXIS FOUND" << std::endl;
            // Seperating axis found
            delete cres;
            return res;
        }
        else {
            //std::cout << "NOT A SEPARATING AXIS" << std::endl;
            //else if this axe is less distance to answer collision
            if (cres->penetrationDistance < res->penetrationDistance) {
                res->pointCollision = test[i];
                res->penetrationDistance = cres->penetrationDistance;
            }
        }
        delete cres;
    }
    //std::cout << "===== AABBOBB COL FIN" << std::endl;
    // Seperating axis not found
    //We have an intersection here
    res->isInCollision = true;
    return res;
}*/

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
    if (compareWithEpsilon(nAB, 0)) {
        return false;
    }

    float t = (distance_to_origin_plane - nA) / nAB;
    if (t >= 0.0f && t <= 1.0f) {
        if (outPoint != 0) {
            *outPoint = start_line + ab * t;
        }
        return true;
    }

    return false;
}

std::vector<glm::vec3> clipEdgesToOBB(const std::vector<glm::vec3>& start_edges, const std::vector<glm::vec3>& end_edges, const Collider& obb) {
    std::vector<glm::vec3> result;
    result.reserve(start_edges.size() * 3);
    
    glm::vec3 intersection;

    std::vector<glm::vec3> normals_plane;
    std::vector<float> distances_to_origin;
    getPlanesOBB(obb, normals_plane,distances_to_origin);

    for (int i = 0; i < normals_plane.size(); ++i) {
        for (int j = 0; j < start_edges.size(); ++j) {
            if (clipToPlane(normals_plane[i], distances_to_origin[i], start_edges[j], end_edges[j], &intersection)) {
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

ColliderResult* OBBOBBCollision(const Collider& obb1, const Collider& obb2) {
    assert(obb1.type == colliderType::OBB);
    assert(obb2.type == colliderType::OBB);
    ColliderResult* res = new ColliderResult();


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

    for (int i = 0; i < 15; ++i) {
        //No need to treat zero sized vectors
        if (glm::dot(test[i], test[i]) < 0.001f) {
            continue;
        }

        float depth = penetrationDepthOBB(obb1, obb2, test[i], &shouldFlip);
        if (depth <= 0.0f) {
            //We found a separation axis
            return res;
        }

        else if (depth < res->penetrationDistance) {
            if (shouldFlip) {
                test[i] = test[i] * -1.0f;
            }
            res->penetrationDistance = depth;
            hitNormal = &test[i];
        }
    }


    if (hitNormal == 0) {
        //No axis found 
        return res;
    }
    glm::vec3 axis = glm::normalize(*hitNormal);

    std::vector<glm::vec3> start_edges_2;
    std::vector<glm::vec3> end_edges_2;

    std::vector<glm::vec3> start_edges_1;
    std::vector<glm::vec3> end_edges_1;
    
    getEdgesOBB(obb1, start_edges_1, end_edges_1);
    getEdgesOBB(obb2, start_edges_2, end_edges_2);
    std::vector<glm::vec3> c1 = clipEdgesToOBB(start_edges_2,end_edges_2, obb1);
    std::vector<glm::vec3> c2 = clipEdgesToOBB(start_edges_1, end_edges_1, obb2);

    std::vector<glm::vec3> pContacts;

    pContacts.reserve(c1.size() + c2.size());
    pContacts.insert(pContacts.end(), c1.begin(), c1.end());
    pContacts.insert(pContacts.end(), c2.begin(), c2.end());

    Interval i = GetIntervalOBB(obb1, axis);

    float distance = (i.max - i.min) * 0.5f - res->penetrationDistance * 0.5f;
    glm::vec3 pointOnPlane = obb1.position + axis * distance;

    for (int i = pContacts.size() - 1; i >= 0; --i) {
        glm::vec3 contact = pContacts[i];
        pContacts[i] = contact + (axis * glm::dot(axis, pointOnPlane - contact));

        // This bit is in the "There is more" section of the book
        for (int j = pContacts.size() - 1; j > i; --j) {
            glm::vec3 dif = pContacts[j] - pContacts[i];
            if (glm::dot(dif,dif) < 0.0001f) {
                pContacts.erase(pContacts.begin() + j);
                break;
            }
        }
    }

    res->isInCollision = true;
    res->normal = axis;
    return res;
}


//Collision between an OBB and a Plane
//Plane is discribed by its normal and a distance to origin
//TODO TEST
ColliderResult* OBBPlaneCollision(const Collider& obb, const glm::vec3 normal_plan, float distance_to_origin) {
    assert(obb.type == colliderType::OBB);

    ColliderResult* res = new ColliderResult();
    // Project the half extents of the OBB onto the plane normal
    float pLen = 
        obb.size.x * std::abs(glm::dot(normal_plan, obb.orientation[0])) +
        obb.size.y * std::abs(glm::dot(normal_plan, obb.orientation[1])) +
        obb.size.z * std::abs(glm::dot(normal_plan, obb.orientation[2]));
    // Find the distance from the center of the OBB to the plane
    float dist = glm::dot(normal_plan, obb.position) - distance_to_origin;
    // Intersection occurs if the distance falls within the projected side
    if (std::abs(dist) <= pLen) {
        res->isInCollision = true;
        res->penetrationDistance = pLen - std::abs(dist);
        res->normal = (dist < 0) ? -normal_plan : normal_plan;
        res->point = obb.position + (res->normal  * (std::abs(dist) - res->penetrationDistance));
    }
    return res;
}


//Intersection function
ColliderResult* intersect(Collider c1, Collider c2) {
    //std::cout << "INTERSECTION TIME" << std::endl;

    //Compute collisions depend on the collider type
    if (c1.type == colliderType::AABB && c2.type == colliderType::AABB)     return AABBAABBCollision(c1, c2);
    if (c1.type == colliderType::AABB && c2.type == colliderType::Sphere)   return SphereAABBCollision(c1, c2);
    if (c1.type == colliderType::Sphere && c2.type == colliderType::AABB)   return SphereAABBCollision(c2, c1);
    if (c1.type == colliderType::OBB && c2.type == colliderType::AABB)      return AABBOBBCollision(c2, c1);
    if (c1.type == colliderType::AABB && c2.type == colliderType::OBB)      return AABBOBBCollision(c1, c2);
    if (c1.type == colliderType::Sphere && c2.type == colliderType::Sphere) return SphereSphereCollision(c1, c2);
    if (c1.type == colliderType::Sphere && c2.type == colliderType::OBB)    return SphereOBBCollision(c1, c2);
    if (c1.type == colliderType::OBB && c2.type == colliderType::Sphere)    return SphereOBBCollision(c2, c1);
    if (c1.type == colliderType::OBB && c2.type == colliderType::OBB)       return OBBOBBCollision(c2, c1);

    ColliderResult* res = new ColliderResult();
    res->isInCollision = false;
    std::cout << "NO COLLISION TEST FOUND !" << std::endl;
    return res;
}


/*============================ JUST COLLIDER INTERSECTION : NO NORMAL AND POSITION CALC ============================*/
bool OverlapOnAxisAABBOBB(const Collider& aabb, const Collider& obb, const glm::vec3& axis) {
    assert(aabb.type == colliderType::AABB);
    assert(obb.type == colliderType::OBB);
    ColliderResult* res = new ColliderResult();
    res->normal = axis;
    Interval a = GetIntervalAABB(aabb, axis);
    Interval b = GetIntervalOBB(obb, axis);
    return ((b.min <= a.max) && (a.min <= b.max));
 }
bool OverlapOnAxisOBBOBB(const Collider& obb1, const Collider& obb2, const glm::vec3& axis) {
    assert(obb1.type == colliderType::OBB);
    assert(obb2.type == colliderType::OBB);

    ColliderResult* res = new ColliderResult();
    res->normal = axis;
    Interval a = GetIntervalOBB(obb1, axis);
    Interval b = GetIntervalOBB(obb2, axis);
    return ((b.min <= a.max) && (a.min <= b.max));
}

bool isAABBAABBCollision(const Collider& aabb1, const Collider& aabb2) {
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
    assert(sp1.type == colliderType::Sphere);
    assert(sp2.type == colliderType::Sphere);
    float r = sp1.radius + sp2.radius;
    glm::vec3 mvt = sp1.position - sp2.position;
    float distance = glm::length(mvt);
    return distance < r;
}
bool isSphereAABBCollision(const Collider& aabb, const Collider& sphere) {
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
    assert(sphere.type == colliderType::Sphere);
    assert(obb.type == colliderType::OBB);
    glm::vec3 closestPoint = closestPointOBB(obb, sphere.position);
    glm::vec3 dif = sphere.position - closestPoint;
    float dist_squared = glm::dot(dif, dif);
    float radius_squared = sphere.radius * sphere.radius;
    return (dist_squared < radius_squared);
}
bool isSpherePlaneCollision(const Collider& sphere, const glm::vec3 normal_plan, float distance_to_origin) {
    assert(sphere.type == colliderType::Sphere);
    glm::vec3 closestPoint = closestPointPlane(sphere.position, normal_plan, distance_to_origin);
    glm::vec3 dif = sphere.position - closestPoint;
    float dist_squared = glm::dot(dif, dif);
    float radius_squared = sphere.radius * sphere.radius;
    return (dist_squared < radius_squared);
}
bool isAABBOBBCollision(const Collider& aabb, const Collider& obb) {
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
        if (!OverlapOnAxisAABBOBB(aabb, obb, test[i])) {
            // Seperating axis found
            return false;
        }
    }
    return true;
}
bool isAABBPlaneCollision(const Collider& aabb, const glm::vec3 normal_plan, float distance_to_origin) {
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
        if (!OverlapOnAxisOBBOBB(obb1, obb2, test[i])) {
            return false;
        }
    }
    return true;
}
bool isOBBPlaneCollision(const Collider& obb, const glm::vec3 normal_plan, float distance_to_origin) {
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
    // Compute direction of intersection line
    glm::vec3 d = glm::cross(normal_plan_1, normal_plan_2);
    // Check the length of the direction line
    // if the length is 0, no intersection happened
    return !(compareWithEpsilon(glm::dot(d, d), 0));
}

bool isInintersection(Collider c1, Collider c2) {
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





std::vector<ColliderResult*> ColliderSystem::getResultOf(unsigned int entityID) {
    std::map<unsigned short, std::vector<ColliderResult*>>::iterator it = collisionResultMap.find(entityID);
    if (it == collisionResultMap.end())
        return std::vector<ColliderResult*>();
    return it->second;
}