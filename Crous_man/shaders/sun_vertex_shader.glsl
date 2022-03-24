#version 330 core
#define MAX_LIGHTS 16

layout(location = 0) in vec3 vertices_position_modelspace;
layout(location = 1) in vec3 vertices_normals_modelspace;
layout(location = 2) in vec2 vertices_UV;

uniform mat4 uModelMatrix;
uniform mat4 uViewMatrix;
uniform mat4 uProjectionMatrix;
uniform mat4 uNormalMatrix;

uniform vec3 light_position_worldspace[MAX_LIGHTS];

out vec3 normal;
out vec3 lightDirection[MAX_LIGHTS];
out vec3 eyeVector;

out vec2 UV;

void main() {
	UV = vertices_UV;

	normal = vec3(uNormalMatrix * vec4(vertices_normals_modelspace, 0.0));
    vec3 vertex = vec3(uModelMatrix * vec4(vertices_position_modelspace, 1.0));
    for (int i = 0; i < MAX_LIGHTS; i++) {
        lightDirection[i] = vertex - light_position_worldspace[i];
    }
    eyeVector = - vec3(uViewMatrix * uModelMatrix * vec4(vertices_position_modelspace, 0.0));

    gl_Position = uProjectionMatrix * uViewMatrix * vec4(vertex, 1.0);
}

