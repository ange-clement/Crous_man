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

}
ColliderResult::~ColliderResult() {

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
            //std::cout << "result of collision : " << ((resC->isInCollision) ? "TRUE" : "FALSE") << " => (" << i << "," << j << ")" << std::endl;
            collisionResultMap.at(entityID)[j] = resC;
            collisionResultMap.at(entityIDJ)[i] = resC;
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
                std::cout << el->isInCollision << "," << el->penetrationDistance << "{" << el->pointCollision.x << "," << el->pointCollision.y << "," << el->pointCollision.z << "}" << std::endl;
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
    glm::vec3 E = obb.size;// OBB Extents

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

ColliderResult* OverlapOnAxisAABBOBB(const Collider& aabb, const Collider& obb, const glm::vec3& axis) {
    assert(aabb.type == colliderType::AABB);
    assert(obb.type == colliderType::OBB);

    ColliderResult* res = new ColliderResult();
    res->pointCollision = axis;

    Interval a = GetIntervalAABB(aabb, axis);
    Interval b = GetIntervalOBB(obb, axis);

    res->isInCollision = ((b.min <= a.max) && (a.min <= b.max));
    res->penetrationDistance = std::min(a.max-b.min,b.max-a.min);
    return res;
}

ColliderResult* OverlapOnAxisOBBOBB(const Collider& obb1, const Collider& obb2, const glm::vec3& axis) {
    assert(obb1.type == colliderType::OBB);
    assert(obb2.type == colliderType::OBB);

    ColliderResult* res = new ColliderResult();
    res->pointCollision = axis;
    Interval a = GetIntervalOBB(obb1, axis);
    Interval b = GetIntervalOBB(obb2, axis);

    /*std::cout << "INTERVALLES OBB 1" << std::endl;
    print(a);
    std::cout << "INTERVALLES OBB 2" << std::endl;
    print(b);*/

    res->isInCollision = ((b.min <= a.max) && (a.min <= b.max));
    res->penetrationDistance = std::min(a.max - b.min, b.max - a.min);
    return res;
}



/* =================== COLLIDERS COLLISION DETECTION FUNCTIONS ===================*/

//Collision between two AABB
ColliderResult* AABBAABBCollision(const Collider& aabb1, const Collider& aabb2) {
    assert(aabb1.type == colliderType::AABB);
    assert(aabb2.type == colliderType::AABB);

    ColliderResult* res = new ColliderResult();
    res->pointCollision = glm::vec3(0);
    res->penetrationDistance = 0;
    res->isInCollision = false;

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


        float x_factor = (dist_x_1 < dist_x_2) ? 1 : -1;
        float y_factor = (dist_y_1 < dist_y_2) ? 1 : -1;
        float z_factor = (dist_z_1 < dist_z_2) ? 1 : -1;

        float x = (dist_x_1 < dist_x_2) ? dist_x_1 : dist_x_2;
        float y = (dist_y_1 < dist_y_2) ? dist_y_1 : dist_y_2;
        float z = (dist_z_1 < dist_z_2) ? dist_z_1 : dist_z_2;

        if (x <= z && x <= y) {
            res->penetrationDistance = x;
            z = 0;
            y = 0;
        }
        else if (y <= x && y <= z) {
            res->penetrationDistance = y;
            x = 0;
            z = 0;
        }
        else if (z <= y && z <= x) {
            res->penetrationDistance = z;
            x = 0;
            y = 0;
        }
        res->pointCollision = glm::normalize(glm::vec3(x * x_factor, y * y_factor, z * z_factor));
    }
    return res;
}

//Collision between two Sphere
ColliderResult* SphereSphereCollision(Collider sp1, Collider sp2) {
    assert(sp1.type == colliderType::Sphere);
    assert(sp2.type == colliderType::Sphere);

    ColliderResult* res = new ColliderResult();
    res->isInCollision = false;

    glm::vec3 mvt = sp1.position - sp2.position;
    float distance = glm::length(mvt);
    if (distance < (sp1.radius + sp2.radius)) {
        res->isInCollision = true;
        res->penetrationDistance = (sp1.radius - distance) + sp2.radius;
        res->pointCollision = ((distance == 0) ? glm::vec3(0, 1, 0) : glm::normalize(mvt));
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
ColliderResult* SphereAABBCollision(const Collider& aabb, const Collider& sphere) {
    /*std::cout << "INTERSECTION SPHERE AABB" << std::endl;
    
    std::cout << "->SPHERE POS" << std::endl;
    print(sphere.position);
    std::cout << "->SPHERE CENTER" << std::endl;
    print(sphere.center);
    std::cout << "->AABB POS" << std::endl;
    print(aabb.position);
    std::cout << "->AABB CENTER" << std::endl;
    print(aabb.center);
    std::cout << "->AABB size" << std::endl;
    print(aabb.size);*/


    assert(aabb.type == colliderType::AABB);
    assert(sphere.type == colliderType::Sphere);
    

    ColliderResult* res = new ColliderResult();
    res->pointCollision = glm::vec3(0);
    res->penetrationDistance = 0;
    res->isInCollision = false;

    glm::vec3 min;
    glm::vec3 max;
    computeMinMaxAABB(aabb, min, max);


    //std::cout << "min" << std::endl;
    //print(min);
    //std::cout << "max" << std::endl;
    //print(max);

    glm::vec3 difference = sphere.position - aabb.position;

    //std::cout << "DIFFERENCE" << std::endl;
    //print(difference);


    //Get the coord of the point in the sphere at smalest dist from the cube
    float x = std::max(min.x, std::min(difference.x, max.x));
    float y = std::max(min.y, std::min(difference.y, max.y));
    float z = std::max(min.z, std::min(difference.z, max.z));

    //std::cout << "x : " << x << std::endl;
    //std::cout << "y : " << y << std::endl;
    //std::cout << "z : " << z << std::endl;


    difference = (glm::vec3(x, y, z) + aabb.position) - sphere.position;
    float distance = glm::length(difference);


    //std::cout << "DIFFERENCE" << std::endl;
    //print(difference);
    //std::cout << "distance : " << distance << std::endl;
    //std::cout << "radius : " << sphere.radius << std::endl;

    if (distance < sphere.radius) {
        //We got a contact
        res->isInCollision = true;
        //std::cout << " ====== COLLISION FOUND ! ====== " << std::endl;

        //If both are in the same place
        if (distance == 0) {
            //Get min dist to get out of other collider
            glm::vec3 dist = max - min;

            if (dist.y <= dist.x && dist.y <= dist.z) {
                res->penetrationDistance = dist.y + sphere.radius;
                res->pointCollision = glm::vec3(0, 1, 0);
            }
            else if (dist.x <= dist.y && dist.x <= dist.z) {
                res->penetrationDistance = dist.x + sphere.radius;
                res->pointCollision = glm::vec3(1, 0, 0);
            }
            else if (dist.z <= dist.y && dist.z <= dist.x) {
                res->penetrationDistance = dist.z + sphere.radius;
                res->pointCollision = glm::vec3(0, 0, 1);
            }
        }
        else {
            //We need to know the direction of the minimum penetration
            res->pointCollision = perform_direction(difference, res->penetrationDistance, sphere.radius);
        }
    }
    return res;
}


ColliderResult* SphereAABBCollision_bis(const Collider& aabb, const Collider& sphere) {
    assert(aabb.type == colliderType::AABB);
    assert(sphere.type == colliderType::Sphere);

    ColliderResult* res = new ColliderResult();
    res->pointCollision = glm::vec3(0);
    res->penetrationDistance = 0;
    res->isInCollision = false;

    glm::vec3 closestPoint = closestPointAABB(aabb, sphere.position);
    glm::vec3 dif = sphere.position - closestPoint;
    float dist_squared = glm::dot(dif,dif);
    float radius_squared = sphere.radius * sphere.radius;

    if (dist_squared < radius_squared) {
        //We got a collision then
        glm::vec3 closestPointSp    = closestPointSphere(sphere, aabb.position);
        dif = closestPoint - closestPointSp;
        res->pointCollision          = glm::normalize(dif);
        res->penetrationDistance     = glm::length(dif);
        res->isInCollision           = true;
    }
    return res;
}

//Collision between an OBB and a sphere
ColliderResult* SphereOBBCollision(const Collider& sphere, const Collider& obb) {
    assert(sphere.type == colliderType::Sphere);
    assert(obb.type == colliderType::OBB);

    ColliderResult* res = new ColliderResult();
    res->pointCollision = glm::vec3(0);
    res->penetrationDistance = 0;
    res->isInCollision = false;

    glm::vec3 closestPoint = closestPointOBB(obb, sphere.position);
    glm::vec3 dif = sphere.position - closestPoint;
    float dist_squared = glm::dot(dif,dif);
    float radius_squared = sphere.radius * sphere.radius;

    if (dist_squared < radius_squared) {
        //We got a collision then
        glm::vec3 closestPointSp    = closestPointSphere(sphere, obb.position);
        dif = closestPoint - closestPointSp;
        res->pointCollision          = glm::normalize(dif);
        res->penetrationDistance     = glm::length(dif);
        res->isInCollision           = true;
    }
    return res;
}

//Collision between a plane and a sphere
//Plane is discribed by its normal and a distance to origin
//TODO TEST
ColliderResult* SpherePlaneCollision(const Collider& sphere, const glm::vec3 normal_plan, float distance_to_origin) {
    assert(sphere.type == colliderType::Sphere);

    ColliderResult* res = new ColliderResult();
    res->pointCollision = glm::vec3(0);
    res->penetrationDistance = 0;
    res->isInCollision = false;

    glm::vec3 closestPoint = closestPointPlane(sphere.position, normal_plan,distance_to_origin);
    glm::vec3 dif = sphere.position - closestPoint;
    float dist_squared = glm::dot(dif, dif);
    float radius_squared = sphere.radius * sphere.radius;

    if (dist_squared < radius_squared) {
        //We got a collision then       
        res->pointCollision = glm::normalize(dif);
        res->penetrationDistance = glm::length(dif) - sphere.radius;
        res->isInCollision = true;
    }
    return res;
}

//Collision between an AABB and an OBB
//TODO TEST
ColliderResult* AABBOBBCollision(const Collider& aabb, const Collider& obb) {
    /*std::cout << "====== AABBOBB COL ======" << std::endl;
    std::cout << "aabb : " << aabb.entityID << std::endl;
    std::cout << "obb : " << obb.entityID << std::endl;

    std::cout << "obb position" << std::endl;
    print(obb.position);
    std::cout << "obb size" << std::endl;
    print(obb.size);
    std::cout << "obb dimensions" << std::endl;
    print(obb.dimensions);

    std::cout << "aabb position" << std::endl;
    print(aabb.position);*/
    

    assert(aabb.type == colliderType::AABB);
    assert(obb.type == colliderType::OBB);

    ColliderResult* res = new ColliderResult();
    res->pointCollision = glm::vec3(0);
    res->penetrationDistance = FLT_MAX;
    res->isInCollision = false;

    /*print(obb.orientation);
    std::cout << "obb ori c1" << std::endl;
    print(obb.orientation[0]);
    std::cout << "obb ori c2" << std::endl;
    print(obb.orientation[1]);
    std::cout << "obb ori c3" << std::endl;
    print(obb.orientation[2]);*/

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
        }else {
            //std::cout << "NOT A SEPARATING AXIS" << std::endl;
            //print(test[i]);
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
}

//Collision between an AABB and a Plane
//Plane is discribed by its normal and a distance to origin
//TODO TEST
ColliderResult* AABBPlaneCollision(const Collider& aabb, const glm::vec3 normal_plan, float distance_to_origin) {
    assert(aabb.type == colliderType::AABB);

    ColliderResult* res = new ColliderResult();
    res->pointCollision = glm::vec3(0);
    res->penetrationDistance = 0;
    res->isInCollision = false;
    
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
        res->pointCollision = (dist<0)? -normal_plan : normal_plan;
    }
    return res;
}

//Collision between two OBB
//TODO TEST
ColliderResult* OBBOBBCollision(const Collider& obb1, const Collider& obb2) {
    assert(obb1.type == colliderType::OBB);
    assert(obb2.type == colliderType::OBB);

    /*std::cout << "====== OBBOBB COL ======" << std::endl;
    std::cout << "obb1 position" << std::endl;
    print(obb1.position);
    std::cout << "obb1 size" << std::endl;
    print(obb1.size);
    std::cout << "obb1 dimensions" << std::endl;
    print(obb1.dimensions);

    std::cout << "obb2 position" << std::endl;
    print(obb2.position);
    std::cout << "obb2 size" << std::endl;
    print(obb2.size);
    std::cout << "obb2 dimensions" << std::endl;
    print(obb2.dimensions);*/
    

    ColliderResult* res = new ColliderResult();
    res->pointCollision = glm::vec3(0);
    res->penetrationDistance = 0;
    res->isInCollision = false;

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
}

//Collision between an OBB and a Plane
//Plane is discribed by its normal and a distance to origin
//TODO TEST
ColliderResult* OBBPlaneCollision(const Collider& obb, const glm::vec3 normal_plan, float distance_to_origin) {
    assert(obb.type == colliderType::OBB);

    ColliderResult* res = new ColliderResult();
    res->pointCollision = glm::vec3(0);
    res->penetrationDistance = 0;
    res->isInCollision = false;

    // Project the half extents of the AABB onto the plane normal
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
        res->pointCollision = (dist < 0) ? -normal_plan : normal_plan;
    }
    return res;
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
