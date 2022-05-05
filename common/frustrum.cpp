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

#include "frustrum.hpp"

/*
glm::vec3 Intersection(glm::vec3 normal_p1, glm::vec3 normal_p2, glm::vec3 normal_p3, float distance_p1, float distance_p2, float distance_p3) {
	
	glm::mat3 D;
	D[0] = normal_p1;
	D[1] = normal_p2;
	D[2] = normal_p3;

	glm::vec3 A = glm::vec3(-distance_p1, -distance_p2, -distance_p3);

	glm::mat3 Dx = D, Dy = D, Dz = D;
	Dx[0].x = A.x; Dx[] = A.y; Dx._13 = A.z;
	Dy[1].x = A.x; Dy[] = A.y; Dy._23 = A.z;
	Dz[2].x = A.x; Dz[] = A.y; Dz._33 = A.z;

	float detD = Determinant(D);

	if (CMP(detD, 0)) {
		return Point();
	}

	float detDx = Determinant(Dx);
	float detDy = Determinant(Dy);
	float detDz = Determinant(Dz);

	return Point(detDx / detD, detDy / detD, detDz / detD);
	
	/*
	vec3 m1(p1.normal.x, p2.normal.x, p3.normal.x);
	vec3 m2(p1.normal.y, p2.normal.y, p3.normal.y);
	vec3 m3(p1.normal.z, p2.normal.z, p3.normal.z);
	vec3 d(-p1.distance, -p2.distance, -p3.distance);

	vec3 u = Cross(m2, m3);
	vec3 v = Cross(m1, d);
	float denom = Dot(m1, u);
	if (CMP(denom, 0.0f)) {
		return Point();
	}
	Point result;
	result.x = Dot(d, u) / denom;
	result.y = Dot(m3, v) / denom;
	result.z = -Dot(m2, v) / denom;
	return result;
	
}*/
