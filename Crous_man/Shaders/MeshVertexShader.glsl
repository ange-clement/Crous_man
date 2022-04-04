#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform mat4 normal;

out vec2 TexCoord;
out vec3 FragPos;
out vec3 Normal;

void main()
{
    vec4 Pos = model * vec4(aPos, 1.0);
    FragPos = Pos.xyz;
    TexCoord = aTexCoords;
    
    Normal = (normal * vec4(aNormal, 0.0)).xyz;
    
    gl_Position = projection * view * Pos;
}