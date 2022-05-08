#ifndef RAY_HPP
#define RAY_HPP

struct Collider;


class Triangle {
public:
	union {
		struct {
			glm::vec3 a;
			glm::vec3 b;
			glm::vec3 c;
		};
		struct {
			glm::vec3 p1;
			glm::vec3 p2;
			glm::vec3 p3;
		};

		glm::vec3 points[3];
		float values[9];
	};

	Triangle();
	Triangle(const glm::vec3& _p1, const glm::vec3& _p2, const glm::vec3& _p3);
};

typedef struct RaycastResult {
	unsigned short entityIDCollid;
	
	glm::vec3 point;
	glm::vec3 normal;

	float t;

	bool hit;
} RaycastResult;


class Ray {
public :
	glm::vec3 origin;
	glm::vec3 direction;

public:

	Ray();
	Ray(const glm::vec3& o, const glm::vec3& d);
	~Ray();

	bool PointOnRay(const glm::vec3& point);
	glm::vec3 ClosestPoint(const glm::vec3& point);

};

Ray* FromPoints(const glm::vec3& from, const glm::vec3& to);
void FromTriangle(const Triangle& t, glm::vec3& normal_plan, float& distance_plan);

//Line functions
glm::vec3 closestPointLine(const glm::vec3& point, const glm::vec3& start_line, const glm::vec3& end_line);
bool pointOnLine(const glm::vec3& point, const glm::vec3& start_line, const glm::vec3& end_line);

bool LinetestSphere(const Collider& sphere, const glm::vec3& start_line, const glm::vec3& end_line);
bool LinetestPlan(const glm::vec3 normal_plan, float distance_to_origin, const glm::vec3& start_line, const glm::vec3& end_line);
bool LinetestAABB(const Collider& aabb, const glm::vec3& start_line, const glm::vec3& end_line);
bool LinetestOBB(const Collider& obb, const glm::vec3& start_line, const glm::vec3& end_line);
bool LinetestTriangle(const Triangle& triangle, const glm::vec3& start_line, const glm::vec3& end_line);
bool LinetestCollider(const Collider& aabb, const glm::vec3& start_line, const glm::vec3& end_line);


glm::vec3 Barycentric(const glm::vec3& p, const Triangle& t);

//Colliders and ray intersection
bool SphereRaycast(const Collider& sphere, const Ray& ray, RaycastResult* outResult);
bool OBBRaycast(const Collider& obb, const Ray& ray, RaycastResult* outResult);
void ResetRaycastResult(RaycastResult* outResult);
bool AABBRaycast(const Collider& aabb, const Ray& ray, RaycastResult* outResult);
bool PlaneRaycast(const glm::vec3 normal_plan, float distance_to_origin, const Ray& ray, RaycastResult* outResult);
bool TriangleRaycast(const Triangle& triangle, const Ray& ray, RaycastResult* outResult);

bool RayCastCollider(const Collider& collider, const Ray& ray, RaycastResult* outResult);

glm::vec3 unproject(const glm::vec3& viewportPoint, const glm::vec2& viewportOrigin, const glm::vec2& viewportSize, const glm::mat4& view, const glm::mat4& projection);
Ray getPickRay(const glm::vec2& viewportPoint, const glm::vec2& viewportOrigin, const glm::vec2& viewportSize, const glm::mat4& view, const glm::mat4& projection);




void printray(const Ray& r);

#endif