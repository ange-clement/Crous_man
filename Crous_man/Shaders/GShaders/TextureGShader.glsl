#version 330 core

layout (location = 0) out vec3 gPosition;
layout (location = 1) out vec3 gNormal;
layout (location = 2) out vec4 gAlbedo;

uniform sampler2D uDiffuseTexture;
uniform sampler2D uSpecularTexture;

in vec2 TexCoords;
in vec3 FragPos;
in vec3 Normal;

void main()
{
    gPosition   = FragPos;
    gNormal     = normalize(Normal);
    gAlbedo.rgb = pow(texture(uDiffuseTexture , TexCoords).rgb, vec3(2.2f));//vec4(.408, .087, .915, 10.0);
    gAlbedo.a   = texture(uSpecularTexture, TexCoords).r;
}