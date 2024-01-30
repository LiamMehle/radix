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
struct PersistantRenderState {
    std::chrono::microseconds sleep_duration_adjustment, target_frametime;
    GLuint perimeter_vao, perimeter_vbo, point_cloud_vao;
    FullProgram point_cloud_program, perimeter_program;
    GLFWwindow* window;
    std::vector<struct CursorPosition> click_points;
    bool left_mouse_was_pressed;
    std::array<GLBufferObject, 2> vbos;
    std::chrono::_V2::steady_clock::time_point t0;
    uint_fast8_t current_active_buffer_id;
};