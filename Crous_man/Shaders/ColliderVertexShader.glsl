#version 330 core

layout(location = 0) in vec3 vertex;

uniform mat4 u_modMat;
uniform mat4 u_viewMat;
uniform mat4 u_projMat;
uniform vec3 u_size;

void main(){
    gl_Position = u_projMat * u_viewMat * u_modMat * vec4(vertex * u_size, 1.0);
}