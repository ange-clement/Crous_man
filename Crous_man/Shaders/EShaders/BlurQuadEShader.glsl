#version 330 core

layout (location = 0) out vec4 texBuffer0;

in vec2 TexCoords;

uniform sampler2D uTextureBuffer;

void main()
{
    //TODO kernel
    vec3 color = vec3(0.0);
    vec2 texelSize = 1.0 / textureSize(uTextureBuffer, 0);
    for(int x = -1; x <= 1; ++x)
    {
        for(int y = -1; y <= 1; ++y)
        {
            vec3 texelColor = texture(uTextureBuffer, TexCoords.xy + vec2(x, y) * texelSize).rgb;
            color += texelColor;
        }    
    }
    color /= 9.0;

    //texBuffer0.rgba = vec4(vec3(color), 1.0);
    texBuffer0.rgba = vec4(vec3(color), 1.0);
}