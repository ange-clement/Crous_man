#version 330 core
#define MAX_LIGHTS 16

uniform int numLights;

in vec3 normal;
in vec3 lightDirection[MAX_LIGHTS];
in vec3 eyeVector;

out vec3 color;

in float debug;

float squareLength(vec3 v) {
    return v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
}

void main(){
    float myDebug = 0;

    vec4 uMaterialDiffuse = vec4(.408, .087, .915, 1.0);
    float uShininess = 5.0;

    vec4 Ia = vec4(0.2) * uMaterialDiffuse;
    vec4 Id = vec4(0.0, 0.0, 0.0, 1.0);
    vec4 Is = vec4(0.0, 0.0, 0.0, 1.0);
    vec3 E = normalize(eyeVector);
    for (int i = 0; i < numLights; i++) {
        vec3 L = normalize(lightDirection[i]);
        vec3 N = normalize(normal);

        float distanceTerm = min(1.0 / squareLength(lightDirection[i])*100, 1.0);
        float lambertTerm = dot(N, -L);
        float useLambertTerm = step(0.0, lambertTerm);

        Id += uMaterialDiffuse * lambertTerm * distanceTerm * useLambertTerm;
        vec3 R = reflect(L, N);
        float specular = pow( max(dot(R, E), 0.0), uShininess) * useLambertTerm;
        Is += vec4(.5) * specular * distanceTerm * useLambertTerm;
    }

    color = vec3(Ia + Id + Is);

    color.r += debug + myDebug;
}