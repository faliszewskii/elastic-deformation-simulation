// Geometry shader for normal visualization
#version 330 core

layout(triangles) in;
layout(line_strip, max_vertices = 2) out;

in vec3 normal[]; // Interpolated normals from vertex shader

out vec3 fragNormal; // To be used in fragment shader for visualization

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
    float normalLength = 0.05f;
    // Compute triangle center
    vec3 center = (gl_in[0].gl_Position.xyz + gl_in[1].gl_Position.xyz + gl_in[2].gl_Position.xyz) / 3.0;

    // Compute the averaged normal (Phong interpolation per triangle)
    vec3 avgNormal = normalize(normal[0] + normal[1] + normal[2]);

    // Transform the center and the normal
    vec4 center_world = model * vec4(center, 1.0);
    vec4 normal_tip_world = model * vec4(center + avgNormal * normalLength, 1.0);

    // Emit the center point
    gl_Position = projection * view * center_world;
    fragNormal = avgNormal;
    EmitVertex();

    // Emit the tip of the normal vector
    gl_Position = projection * view * normal_tip_world;
    fragNormal = avgNormal;
    EmitVertex();

    EndPrimitive();
}