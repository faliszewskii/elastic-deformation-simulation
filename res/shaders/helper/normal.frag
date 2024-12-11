#version 400 core
out vec4 FragColor;

in vec3 fragNormal;

void main()
{
    FragColor = vec4(fragNormal, 1);
}