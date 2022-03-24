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

	vec4 uMaterialDiffuse = vec4(textureColor, 1.0);

    vec4 Ia = vec4(textureColor*0.2, 1.0);
    vec4 Id = vec4(0.0, 0.0, 0.0, 1.0);
    vec3 E = normalize(eyeVector);
    for (int i = 0; i < numLights; i++) {
        vec3 L = normalize(lightDirection[i]);
        vec3 N = normalize(normal);

        float distanceTerm = min(1.0 / squareLength(lightDirection[i])*100, 1.0);
        float lambertTerm = dot(N, -L);
        float useLambertTerm = step(0.0, lambertTerm);

        Id += uMaterialDiffuse * lambertTerm * useLambertTerm;
    }

    color = vec3(Ia + Id*2.0);
}
