#version 330 core
#define MAX_LIGHTS 16

uniform int numLights;

uniform sampler2D grassTexture;
uniform sampler2D rockTexture;
uniform sampler2D snowrockTexture;

uniform float textureScale;


in vec3 normal;
in vec3 lightDirection[MAX_LIGHTS];
in vec3 eyeVector;

in float height;
in vec2 UV;

in float debug;

out vec3 color;

float squareLength(vec3 v) {
    return v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
}

void main(){
	vec2 coordUV = UV*textureScale;

	float smoothS = 0.1;
	float grassCut = 0.35;
	float rockCut = 0.6;

	float grass = 1.0-smoothstep(grassCut-smoothS, grassCut, height);
	float rock = 1.0 - grass - smoothstep(rockCut-smoothS, rockCut, height);
	float snow = smoothstep(rockCut-smoothS, rockCut, height);

	vec3 grassColor = texture(grassTexture, coordUV).rgb;
	vec3 rockColor = texture(rockTexture, coordUV).rgb;
	vec3 snowrockColor = texture(snowrockTexture, coordUV).rgb;

	vec3 textColor = grass * grassColor;
	textColor += rock * rockColor;
	textColor += snow * snowrockColor;
	textColor *= height*0.5+0.5;


	vec4 uMaterialDiffuse = vec4(textColor, 1.0);

    vec4 Ia = vec4(0.2) * uMaterialDiffuse;
    vec4 Id = vec4(0.0, 0.0, 0.0, 1.0);
    vec3 E = normalize(eyeVector);
    for (int i = 0; i < numLights; i++) {
        vec3 L = normalize(lightDirection[i]);
        vec3 N = normalize(normal);

        float distanceTerm = min(1.0 / squareLength(lightDirection[i])*10000, 1.0);
        float lambertTerm = dot(N, -L);
        float useLambertTerm = step(0.0, lambertTerm);

        Id += uMaterialDiffuse * lambertTerm * distanceTerm * useLambertTerm;
    }

    color = vec3(Ia + Id);

	//color = normal.xyz;
}
