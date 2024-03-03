#version 330 core

layout (triangles) in;
layout (triangle_strip, max_vertices=3) out;
in float heights[];
out float height;

vec4 vertex_at(int idx) {
    return gl_in[idx].gl_Position;
}

void copy_vertex(int idx) {
    gl_Position = vertex_at(idx);
    height = heights[idx];
    EmitVertex();
}

vec3 vertex3_at(int idx) {
    return vertex_at(idx).xyz;
}

void main() {
    vec3 edge1 = normalize(vertex3_at(1) - vertex3_at(0));
    vec3 edge2 = normalize(vertex3_at(2) - vertex3_at(0));
    vec3 edge3 = normalize(vertex3_at(1) - vertex3_at(2));
    vec3 lengths = vec3(
        abs(dot(edge1, edge2)),
        abs(dot(edge1, edge3)),
        abs(dot(edge2, edge3))
    );
    float max_length = max(lengths.x, max(lengths.y, lengths.z));
    bool is_splinter = max_length > .95f;
    if (is_splinter)
        return;

    copy_vertex(0);
    copy_vertex(1);
    copy_vertex(2);
    EndPrimitive();
}