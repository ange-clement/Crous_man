#version 330 core

layout (location = 0) out vec4 texBuffer0;

in vec2 TexCoords;
in vec3 FragPos;
in vec3 Normal;

uniform vec3 uFromPos;

void main()
{
    //texBuffer0.r = length(FragPos - uFromPos);
    //texBuffer0.rgba = vec4(0.5, 0.5, 0.5, 1.0);
    texBuffer0.rgba = vec4(vec3(length(FragPos - uFromPos)), 1.0);
}