#version 330 core

layout (location = 0) out vec4 texBuffer0;

in vec2 TexCoords;
in vec3 FragPos;
in vec3 Normal;

uniform vec3 uFromPos;
uniform float uMaxDistance;

float LinearizeDepth(float depth)
{
    float z = depth * 2.0 - 1.0; // Back to NDC 
    return (2.0 * 1.0 * uMaxDistance) / (uMaxDistance + 1.0 - z * (uMaxDistance - 1.0));
}

void main()
{
    texBuffer0.rgba = vec4(vec3(length(FragPos - uFromPos)) / uMaxDistance, 1.0);
    //texBuffer0.rgba = vec4(vec3(LinearizeDepth(length(FragPos - uFromPos))) / uMaxDistance, 1.0);
}