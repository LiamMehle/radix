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
    GLuint vao     = 0;
    GLint  program = 0;
    GLenum draw_mode = GL_TRIANGLES;
    GLuint vertex_offset = 0;
    GLuint vertex_count = 0;
    uint32_t uniform_count = 0;
    uint32_t texture_unit_count = 0;
    UniformConfig uniforms[UNIFORM_COUNT];
    TextureUnitConfig texture_units[TEXTURE_UNIT_COUNT];
    void draw() const {
        // bind Vertex data
        glBindVertexArray(this->vao);
        glUseProgram(this->program);

        // configure uniforms
        for (int i = 0; i < this->uniform_count; i++) {
            switch (this->uniforms[i].type) {
                case UniformConfig::UniformType::Integer1:
                    glUniform1i(this->uniforms[i].location, this->uniforms[i].value.integer);
            }
        }
        // configure textures
        for (int i = 0; i < this->texture_unit_count; i++) {
            glActiveTexture(GL_TEXTURE0 + i);  // selects the active texture UNIT, not texture itself
            glBindTexture(GL_TEXTURE_2D, this->texture_units[i].assigned_texture);
        }
        // draw
        glDrawArrays(this->draw_mode, static_cast<GLint>(this->vertex_offset),
                     static_cast<GLint>(this->vertex_count));
        glBindVertexArray(0);
    }
};
struct PrivateRenderData {
    DrawCallInfo point_cloud;
    DrawCallInfo perimeter;
    std::vector<std::optional<DrawCallInfo>> drawables;
};
struct TextRenderResource {
    FullProgram program;
    GLuint vao;
    GLint sampler_uniform_location;
    GLint pitch_uniform_location;
    Charset<128> small_charset;
};
struct DrawCallCleanupInfo {
    GLuint vao,vbo;
};
struct PersistentRenderState {
    std::chrono::microseconds sleep_duration_adjustment, target_frame_time;
    GLuint perimeter_vao, perimeter_vbo, point_cloud_vao;
    FullProgram point_cloud_program, perimeter_program;
    GLFWwindow* window;
    std::vector<struct CursorPosition> click_points;
    std::vector<struct DrawCallCleanupInfo> objects_for_cleanup;
    bool left_mouse_was_pressed;
    std::array<GLBufferObject, 2> vertex_buffer_objects;
    std::chrono::_V2::steady_clock::time_point t0;
    uint_fast8_t current_active_buffer_id;
    TextRenderResource text_rendering_resource;
};
template<typename F>
class Defer {
    F f;
public:
    explicit Defer(F&& f) : f(f) {}
    ~Defer() { f(); }
};
class VertexArrayList {
    std::vector<GLuint> list;
    std::vector<bool>   is_in_use;
public:
    VertexArrayList() = default;
    explicit VertexArrayList(std::size_t count) {
        list = std::vector<GLuint>(count);
        glCreateVertexArrays(list.size(), list.data());
    }
    ~VertexArrayList() {
        if (!list.empty())
            glDeleteVertexArrays(list.size(), list.data());
    }
    VertexArrayList(VertexArrayList const&) = delete;
    VertexArrayList& operator=(VertexArrayList const&) = delete;
    VertexArrayList(VertexArrayList&& other) noexcept {
        this->list = other.list;
        this->is_in_use = other.is_in_use;
        other.list.clear();
        other.is_in_use.clear();
    }
    VertexArrayList& operator=(VertexArrayList&& other) noexcept {
        new(this)VertexArrayList(std::move(other));  // reuse move ctor
        return *this;
    }

    GLuint get() {
        for (int i=0; i<this->list.size(); i++)
            if (!this->is_in_use[i]) {
                this->is_in_use[i] = true;
                return this->list[i];
            }

        // dang, we're out of vertex arrays
        auto const current_size = this->list.size();
        auto const new_size = current_size ? 2*current_size : 1;
        this->list.reserve(new_size);
        this->is_in_use.reserve(new_size);
        for (int i=current_size; i<this->list.size(); i++) {
            this->list.emplace_back(0);
            this->is_in_use.emplace_back(false);
        }
        glGenVertexArrays(new_size-current_size, &this->list[current_size]);
        this->is_in_use[current_size] = true;
        return this->list[current_size];
    }
    void retire(GLuint const object) {
        for (int i=0; i<this->list.size(); i++)
            if (this->list[i] == object) {
                this->is_in_use[i] = false;
                break;
            }
    }
};