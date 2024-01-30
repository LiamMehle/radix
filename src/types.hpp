#pragma once
// data used for rendering of a frame
enum PrivateRenderDataFlagBits {
    perimeter_enabled = 1,
};
struct DrawCallInfo {
    GLenum draw_mode;
    GLuint vao;
    GLuint vertex_offset;
    GLuint vertex_count;
};
struct PrivateRenderData {
    uint32_t flags;
    DrawCallInfo point_cloud_draw_info;
    DrawCallInfo perimeter_draw_info;
};
struct SizedVbo {
    GLuint vbo;
    GLint vertex_count;
};