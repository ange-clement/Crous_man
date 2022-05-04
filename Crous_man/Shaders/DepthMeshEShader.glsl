#version 330 core

layout (location = 0) out vec4 texBuffer0;

in vec2 TexCoords;
in vec3 FragPos;
in vec3 Normal;

uniform vec3 uFromPos;
uniform float uMaxDistance;

void main()
{
    texBuffer0.rgba = vec4(vec3(length(FragPos - uFromPos)) / uMaxDistance, 1.0);
}