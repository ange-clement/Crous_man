#version 330 core

layout(location = 0) in vec3 vertex;

uniform mat4 u_modMat;
uniform mat4 u_viewMat;
uniform mat4 u_projMat;

void main(){
    gl_Position = u_projMat * u_viewMat * u_modMat * vec4(vertex, 1.0);
}