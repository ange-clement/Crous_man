#ifndef FRUSTRUM_CAMERA_HPP
#define FRUSTRUM_CAMERA_HPP

typedef struct Frustum {

	glm::vec3 top_normal;
	float distance_to_origin_top_plane;
	glm::vec3 bottom;
	float distance_to_origin_bottom_plane;
	glm::vec3 left;
	float distance_to_origin_left_plane;
	glm::vec3 right;
	float distance_to_origin_right_plane;
	glm::vec3 _near;
	float distance_to_origin_near_plane;
	glm::vec3 _far;
	float distance_to_origin_far_plane;
	
	inline Frustum() { }
} Frustum;


#endif