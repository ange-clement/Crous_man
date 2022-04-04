#version 330 core

in vec2 TexCoords;

uniform sampler2D gPosition;
uniform sampler2D gNormal;
uniform sampler2D gAlbedo;
/*
struct Light {
    vec3 Position;
    vec3 Color;
    
    float Linear;
    float Quadratic;
};
uniform Light light;
*/
uniform vec3 viewPos;
out vec4 FragColor;

void main()
{
    vec3 FragPos = texture(gPosition, TexCoords).rgb;
    vec3 Normal = texture(gNormal, TexCoords).rgb;
    vec3 Diffuse = texture(gAlbedo, TexCoords).rgb;
    
    /*
    vec3 ambient = vec3(0.3 * Diffuse);
    vec3 lighting  = ambient;
    vec3 viewDir  = normalize(-FragPos);
    // diffuse
    vec3 lightDir = normalize(light.Position - FragPos);
    vec3 diffuse = max(dot(Normal, lightDir), 0.0) * Diffuse * light.Color;
    // specular
    vec3 halfwayDir = normalize(lightDir + viewDir);  
    float spec = pow(max(dot(Normal, halfwayDir), 0.0), 8.0);
    vec3 specular = light.Color * spec;
    // attenuation
    float distance = length(light.Position - FragPos);
    float attenuation = 1.0 / (1.0 + light.Linear * distance + light.Quadratic * distance * distance);
    diffuse *= attenuation;
    specular *= attenuation;
    lighting += diffuse + specular;

    FragColor = vec4(lighting, 1.0);
    */

    vec3 ambient = vec3(0.3 * Diffuse);
    vec3 lighting  = ambient;
    vec3 viewDir  = normalize(viewPos-FragPos);
    // diffuse
    vec3 lightDir = normalize(vec3(0.0, 3.0, 0.0) - FragPos);
    vec3 diffuse = max(dot(Normal, lightDir), 0.0) * Diffuse * vec3(1.0, 1.0, 1.0);
    // specular
    vec3 halfwayDir = normalize(lightDir + viewDir);  
    float spec = pow(max(dot(Normal, halfwayDir), 0.0), 8.0);
    vec3 specular = vec3(1.0, 1.0, 1.0) * spec;
    lighting += diffuse + specular;

    FragColor = vec4(lighting, 1.0);
}