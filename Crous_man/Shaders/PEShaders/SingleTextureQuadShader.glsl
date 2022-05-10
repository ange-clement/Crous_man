#version 330 core

out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D texture2D;

void main()
{
    FragColor = vec4(pow(texture(texture2D, TexCoords).rgb, vec3(1.0/2.2)), 1.0);
    //FragColor = vec4(texture(texture2D, TexCoords).rgb, 1.0);
}