#version 330 core
#define MAX_LIGHTS 16

uniform int numLights;

uniform sampler2D texture;

out vec3 color;

in vec3 normal;
in vec3 eyeVector;

in vec2 UV;
in vec3 lightDirection[MAX_LIGHTS];

float squareLength(vec3 v) {
    return v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
}

void main(){
	vec3 textureColor = texture2D(texture, UV).rgb;

    color = textureColor;
}
