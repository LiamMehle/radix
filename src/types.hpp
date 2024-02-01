#pragma once
// data used for rendering of a frame
struct UniformConfig {
    enum UniformType {
        Integer1
    };
    union Value {
        GLint integer;
    };
    UniformType type;
    GLint location;
    Value value;
};
struct TextureUnitConfig {
    GLuint assigned_texture;
};
struct DrawCallInfo {
    GLuint vao;
    GLenum draw_mode;
    GLint vertex_offset;
    GLint vertex_count;
    uint32_t uniform_count;
    UniformConfig uniforms[UNIFORM_COUNT];
    uint32_t texture_unit_count;
    TextureUnitConfig texture_units[TEXTURE_UNIT_COUNT];
};
struct PrivateRenderData {
    std::vector<std::optional<DrawCallInfo>> drawables;
};
struct TextRenderResource {
    FullProgram program;
    GLuint vao;
    GLint sampler_uniform_location;
    GLint pitch_uniform_location;
    Charset<128> big_charset;
    Charset<128> small_charset;
};
struct DrawCallCleanupInfo {
    GLuint vao,vbo;
};
struct PersistentRenderState {
    std::chrono::microseconds sleep_duration_adjustment, target_frametime;
    GLuint perimeter_vao, perimeter_vbo, point_cloud_vao;
    FullProgram point_cloud_program, perimeter_program;
    GLFWwindow* window;
    std::vector<struct CursorPosition> click_points;
    std::vector<struct DrawCallCleanupInfo> objects_for_cleanup;
    bool left_mouse_was_pressed;
    std::array<GLBufferObject, 2> vbos;
    std::chrono::_V2::steady_clock::time_point t0;
    uint_fast8_t current_active_buffer_id;
    TextRenderResource text_rendering_resource;
};