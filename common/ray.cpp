#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>
#include <Crous_man/Util.hpp>
#include <Crous_man/Components/Collider.hpp>

#include "ray.hpp"


/* =============================== TRIANGLES  =============================== */
Triangle::Triangle(){
}

Triangle::Triangle(const glm::vec3& _p1, const glm::vec3& _p2, const glm::vec3& _p3) : a(_p1), b(_p2), c(_p3) {
}


bool LinetestTriangle(const Triangle& triangle, const glm::vec3& start_line, const glm::vec3& end_line) {
	Ray ray = Ray(start_line, end_line - start_line);

	
	RaycastResult raycast;
	if (!TriangleRaycast(triangle, ray, &raycast)) {
		return false;
	}
	float t = raycast.t;

	return t >= 0 && t * t <= glm::dot(end_line - start_line, end_line - start_line);
}

void FromTriangle(const Triangle& t, glm::vec3& normal_plan, float& distance_plan) {
	normal_plan = glm::normalize(glm::cross(t.b - t.a, t.c - t.a));
	distance_plan = glm::dot(normal_plan, t.a);
}

glm::vec3 Barycentric(const glm::vec3& p, const Triangle& t) {
	glm::vec3 ap = p - t.a;
	glm::vec3 bp = p - t.b;
	glm::vec3 cp = p - t.c;

	glm::vec3 ab = t.b - t.a;
	glm::vec3 ac = t.c - t.a;
	glm::vec3 bc = t.c - t.b;
	glm::vec3 cb = t.b - t.c;
	glm::vec3 ca = t.a - t.c;

	glm::vec3 v = ab - Project(ab, cb);
	float a = 1.0f - (glm::dot(v, ap) / glm::dot(v, ab));

	v = bc - Project(bc, ac);
	float b = 1.0f - (glm::dot(v, bp) / glm::dot(v, bc));

	v = ca - Project(ca, ab);
	float c = 1.0f - (glm::dot(v, cp) / glm::dot(v, ca));

	return glm::vec3(a, b, c);
}

/* ================================= RAY ================================= */
Ray* FromPoints(const glm::vec3& from, const glm::vec3& to) {
	return new Ray(from, to - from);
}

Ray::Ray() : direction(glm::vec3(0.0f, 0.0f, 1.0f)) {}

Ray::Ray(const glm::vec3& o, const glm::vec3& d) : origin(o), direction(d){
	glm::normalize(direction);
}

Ray::~Ray() {

}


bool Ray::PointOnRay(const glm::vec3& point) {
	if (point == this->origin) {
		return true;
	}

	glm::vec3 norm = point -this->origin;
	glm::normalize(norm);
	float diff = glm::dot(norm, this->direction);
	return compareWithEpsilon(diff, 1.0f);
}

glm::vec3 Ray::ClosestPoint(const glm::vec3& point) {
	// Project point onto ray, 
	float t = glm::dot(point - this->origin, this->direction);

	// We only want to clamp t in the positive direction.
	t = std::max(t, 0.0f);
	
	return glm::vec3(this->origin + this->direction * t);
}


/*LINE TEST RAY BASED FUNCTIONS*/
glm::vec3 closestPointLine(const glm::vec3& point, const glm::vec3& start_line, const glm::vec3& end_line) {
	glm::vec3 lVec = end_line - start_line;
	float t = glm::dot(point - start_line, lVec) / glm::dot(lVec, lVec);
	// Clamp t to the 0 to 1 range
	t = ((t < 0.0f) ? 0.0f : ((t > 1.0f) ? 1.0f : t));
	// Return projected position of t
	return start_line + lVec * t;
}
bool pointOnLine(const glm::vec3& point, const glm::vec3& start_line, const glm::vec3& end_line) {
	glm::vec3 closest = closestPointLine(point, start_line, end_line);
	float distanceSq = glm::length(closest - point);
	return compareWithEpsilon(distanceSq, 0.0f);
}


bool LinetestSphere(const Collider& sphere, const glm::vec3& start_line, const glm::vec3& end_line) {
	glm::vec3 closest = closestPointLine(sphere.position,start_line,end_line);
	glm::vec3 dif = sphere.position - closest;
	float distSq = glm::dot(dif, dif);
	return distSq <= (sphere.radius * sphere.radius);
}

bool LinetestPlan(const glm::vec3 normal_plan, float distance_to_origin, const glm::vec3& start_line, const glm::vec3& end_line) {
	glm::vec3 ab = end_line - start_line;

	float nA = glm::dot(normal_plan, start_line);
	float nAB = glm::dot(normal_plan, ab);

	if (compareWithEpsilon(nAB, 0)) {
		return false;
	}

	float t = (distance_to_origin - nA) / nAB;
	return t >= 0.0f && t <= 1.0f;
}

bool LinetestAABB(const Collider& aabb, const glm::vec3& start_line, const glm::vec3& end_line) {
	assert(aabb.type == colliderType::AABB);
	Ray ray = Ray(start_line,end_line - start_line);
	
	RaycastResult raycast;
	if (!AABBRaycast(aabb, ray, &raycast)) {
		return false;
	}
	float t = raycast.t;

	glm::vec3 dif = end_line - start_line;
	return t >= 0 && t * t <= glm::dot(dif,dif);
}

bool LinetestOBB(const Collider& obb, const glm::vec3& start_line, const glm::vec3& end_line) {
	assert(obb.type == colliderType::OBB);

	glm::vec3 dif = end_line - start_line;
	float length_sq = glm::dot(dif, dif);
	if (length_sq < 0.0000001f) {
		return pointInOBB(start_line, obb);
	}
	Ray ray = Ray(start_line, glm::normalize(end_line- start_line));

	RaycastResult result;
	if (!OBBRaycast(obb, ray, &result)) {
		return false;
	}
	float t = result.t;
	return t >= 0 && t * t <= length_sq;
}


/*============ =============== COLLIDERS-RAY INTERSECTION FUNCTIONS =============== ============*/
bool SphereRaycast(const Collider& sphere, const Ray& ray, RaycastResult* outResult) {
	assert(sphere.type == colliderType::Sphere);
	ResetRaycastResult(outResult);

	glm::vec3 e = sphere.position - ray.origin;
	float rSq = sphere.radius * sphere.radius;

	float eSq = glm::dot(e,e);
	float a = glm::dot(e, ray.direction); // ray.direction is assumed to be normalized
	float bSq = eSq - (a * a);
	float f = std::sqrt(fabsf((rSq)- bSq));

	// Assume normal intersection!
	float t = a - f;

	// No collision has happened
	if (rSq - (eSq - a * a) < 0.0f) {
		return false;
	}
	// Ray starts inside the sphere
	else if (eSq < rSq) {
		// Just reverse direction
		t = a + f;
	}
	if (outResult != 0) {
		outResult->t = t;
		outResult->hit = true;
		outResult->point = ray.origin + ray.direction * t;
		outResult->normal = glm::normalize(outResult->point - sphere.position);
	}
	return true;
}

bool OBBRaycast(const Collider& obb, const Ray& ray, RaycastResult* outResult) {
	assert(obb.type == colliderType::OBB);
	ResetRaycastResult(outResult);

	glm::vec3 p = obb.position - ray.origin;

	glm::vec3 X = obb.orientation[0];
	glm::vec3 Y = obb.orientation[1];
	glm::vec3 Z = obb.orientation[2];
	
	glm::vec3 f(
		glm::dot(X, ray.direction),
		glm::dot(Y, ray.direction),
		glm::dot(Z, ray.direction)
	);

	glm::vec3 e(
		glm::dot(X, p),
		glm::dot(Y, p),
		glm::dot(Z, p)
	);

	float t[6] = { 0, 0, 0, 0, 0, 0 };

	for (int i = 0; i < 3; ++i) {
		if (compareWithEpsilon(f[i], 0)) {
			if (-e[i] - obb.size[i] > 0 || -e[i] + obb.size[i] < 0) {
				return false;
			}
			f[i] = 0.00001f; // Avoid div by 0!
		}

		t[i * 2 + 0] = (e[i] + obb.size[i]) / f[i]; // tmin[x, y, z]
		t[i * 2 + 1] = (e[i] - obb.size[i]) / f[i]; // tmax[x, y, z]
	}

	float tmin = fmaxf(fmaxf(fminf(t[0], t[1]), fminf(t[2], t[3])), fminf(t[4], t[5]));
	float tmax = fminf(fminf(fmaxf(t[0], t[1]), fmaxf(t[2], t[3])), fmaxf(t[4], t[5]));

	/*
	// The above loop simplifies the below if statements
	// this is done to make sure the sample fits into the book
	if (CMP(f.x, 0)) {
		if (-e.x - obb.size.x > 0 || -e.x + obb.size.x < 0) {
			return -1;
		}
		f.x = 0.00001f; // Avoid div by 0!
	}
	else if (CMP(f.y, 0)) {
		if (-e.y - obb.size.y > 0 || -e.y + obb.size.y < 0) {
			return -1;
		}
		f.y = 0.00001f; // Avoid div by 0!
	}
	else if (CMP(f.z, 0)) {
		if (-e.z - obb.size.z > 0 || -e.z + obb.size.z < 0) {
			return -1;
		}
		f.z = 0.00001f; // Avoid div by 0!
	}
	float t1 = (e.x + obb.size.x) / f.x;
	float t2 = (e.x - obb.size.x) / f.x;
	float t3 = (e.y + obb.size.y) / f.y;
	float t4 = (e.y - obb.size.y) / f.y;
	float t5 = (e.z + obb.size.z) / f.z;
	float t6 = (e.z - obb.size.z) / f.z;
	float tmin = fmaxf(fmaxf(fminf(t1, t2), fminf(t3, t4)), fminf(t5, t6));
	float tmax = fminf(fminf(fmaxf(t1, t2), fmaxf(t3, t4)), fmaxf(t5, t6));
	*/

	// if tmax < 0, ray is intersecting AABB
	// but entire AABB is behing it's origin
	if (tmax < 0) {
		return false;
	}

	// if tmin > tmax, ray doesn't intersect AABB
	if (tmin > tmax) {
		return false;
	}

	// If tmin is < 0, tmax is closer
	float t_result = tmin;

	if (tmin < 0.0f) {
		t_result = tmax;
	}

	if (outResult != 0) {
		outResult->hit = true;
		outResult->t = t_result;
		outResult->point = ray.origin + ray.direction * t_result;

		glm::vec3 normals[] = {
			X,			// +x
			X * -1.0f,	// -x
			Y,			// +y
			Y * -1.0f,	// -y
			Z,			// +z
			Z * -1.0f	// -z
		};

		for (int i = 0; i < 6; ++i) {
			if (compareWithEpsilon(t_result, t[i])) {
				outResult->normal = glm::normalize(normals[i]);
			}
		}
	}
	return true;
}

void ResetRaycastResult(RaycastResult* outResult) {
	if (outResult != 0) {
		outResult->t = -1;
		outResult->hit = false;
		outResult->normal = glm::vec3(0, 0, 1);
		outResult->point = glm::vec3(0, 0, 0);
	}
}

bool AABBRaycast(const Collider& aabb, const Ray& ray, RaycastResult* outResult) {
	assert(aabb.type == colliderType::AABB);
	ResetRaycastResult(outResult);

	glm::vec3 min;
	glm::vec3 max;
	computeMinMaxAABB(aabb, min, max);

	// Any component of direction could be 0!
	// Address this by using a small number, close to
	// 0 in case any of directions components are 0
	float t1 = (min.x - ray.origin.x) / (compareWithEpsilon(ray.direction.x, 0.0f) ? 0.00001f : ray.direction.x);
	float t2 = (max.x - ray.origin.x) / (compareWithEpsilon(ray.direction.x, 0.0f) ? 0.00001f : ray.direction.x);
	float t3 = (min.y - ray.origin.y) / (compareWithEpsilon(ray.direction.y, 0.0f) ? 0.00001f : ray.direction.y);
	float t4 = (max.y - ray.origin.y) / (compareWithEpsilon(ray.direction.y, 0.0f) ? 0.00001f : ray.direction.y);
	float t5 = (min.z - ray.origin.z) / (compareWithEpsilon(ray.direction.z, 0.0f) ? 0.00001f : ray.direction.z);
	float t6 = (max.z - ray.origin.z) / (compareWithEpsilon(ray.direction.z, 0.0f) ? 0.00001f : ray.direction.z);

	float tmin = fmaxf(fmaxf(fminf(t1, t2), fminf(t3, t4)), fminf(t5, t6));
	float tmax = fminf(fminf(fmaxf(t1, t2), fmaxf(t3, t4)), fmaxf(t5, t6));

	// if tmax < 0, ray is intersecting AABB
	// but entire AABB is behing it's origin
	if (tmax < 0) {
		return false;
	}

	// if tmin > tmax, ray doesn't intersect AABB
	if (tmin > tmax) {
		return false;
	}

	float t_result = tmin;

	// If tmin is < 0, tmax is closer
	if (tmin < 0.0f) {
		t_result = tmax;
	}

	if (outResult != 0) {
		outResult->t = t_result;
		outResult->hit = true;
		outResult->point = ray.origin + ray.direction * t_result;

		glm::vec3 normals[] = {
			glm::vec3(-1, 0, 0),
			glm::vec3(1, 0, 0),
			glm::vec3(0, -1, 0),
			glm::vec3(0, 1, 0),
			glm::vec3(0, 0, -1),
			glm::vec3(0, 0, 1)
		};
		float t[] = { t1, t2, t3, t4, t5, t6 };

		for (int i = 0; i < 6; ++i) {
			if (compareWithEpsilon(t_result, t[i])) {
				outResult->normal = normals[i];
			}
		}
	}

	return true;
}

bool PlaneRaycast(const glm::vec3 normal_plan, float distance_to_origin, const Ray& ray, RaycastResult* outResult) {
	ResetRaycastResult(outResult);

	float nd = glm::dot(ray.direction, normal_plan);
	float pn = glm::dot(ray.origin, normal_plan);

	// nd must be negative, and not 0
	// if nd is positive, the ray and plane normals
	// point in the same direction. No intersection.
	if (nd >= 0.0f) {
		return false;
	}

	float t = (distance_to_origin - pn) / nd;

	// t must be positive
	if (t >= 0.0f) {
		if (outResult != 0) {
			outResult->t = t;
			outResult->hit = true;
			outResult->point = ray.origin + ray.direction * t;
			outResult->normal = glm::normalize(normal_plan);
		}
		return true;
	}

	return false;
}

bool TriangleRaycast(const Triangle& triangle, const Ray& ray, RaycastResult* outResult) {
	ResetRaycastResult(outResult);
	glm::vec3 normal_plan;
	float distance_plan;
	FromTriangle(triangle, normal_plan,distance_plan);

	RaycastResult planeResult;
	if (!PlaneRaycast(normal_plan,distance_plan, ray, &planeResult)) {
		return false;
	}
	float t = planeResult.t;

	glm::vec3 result = ray.origin + ray.direction * t;

	glm::vec3 barycentric = Barycentric(result, triangle);
	if (barycentric.x >= 0.0f && barycentric.x <= 1.0f &&
		barycentric.y >= 0.0f && barycentric.y <= 1.0f &&
		barycentric.z >= 0.0f && barycentric.z <= 1.0f) {

		if (outResult != 0) {
			outResult->t = t;
			outResult->hit = true;
			outResult->point = ray.origin + ray.direction * t;
			outResult->normal = normal_plan;
		}

		return true;
	}

	return false;
}

bool RayCastCollider(const Collider& collider, const Ray& ray, RaycastResult* outResult){
	if (collider.type == colliderType::Sphere)	return SphereRaycast(collider, ray, outResult);
	if (collider.type == colliderType::AABB)	return AABBRaycast(collider, ray, outResult);
	if (collider.type == colliderType::OBB)		return OBBRaycast(collider, ray, outResult);
	return false;
}