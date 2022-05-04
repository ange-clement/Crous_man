#version 330 core

layout (location = 0) out vec4 texBuffer0;

in vec2 TexCoords;

uniform sampler2D gPosition;
uniform sampler2D uShadowMap;
uniform mat4 uLightSpaceMatrix;

void main()
{
    vec3 FragPos = texture(gPosition, TexCoords).rgb;
    vec4 FragPosLightSpace = uLightSpaceMatrix * vec4(FragPos, 1.0);
    // perform perspective divide
    vec3 projCoords = FragPosLightSpace.xyz / FragPosLightSpace.w;
    // transform to [0,1] range
    projCoords = projCoords * 0.5 + 0.5;
    // get closest depth value from light's perspective (using [0,1] range fragPosLight as coords)
    float closestDepth = texture(uShadowMap, TexCoords.xy).r;
    // get depth of current fragment from light's perspective
    float currentDepth = projCoords.z;
    // check whether current frag pos is in shadow
    float shadow = currentDepth > closestDepth  ? 1.0 : 0.0;

    texBuffer0.rgba = vec4(vec3(shadow), 1.0);
    //texBuffer0.rgba = vec4(vec3(currentDepth), 1.0);
    //texBuffer0.rgba = vec4(projCoords, 1.0);
    //texBuffer0.rgba = vec4(vec3(0.5), 1.0);
}