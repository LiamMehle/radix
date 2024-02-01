#pragma once
#include <utility>
#include <optional>
#include <cstdint>
#include <ft2build.h>
#include FT_FREETYPE_H

struct Bitmap {
    GLuint   texture;           // size 4,  align 4
    uint32_t width;             // size 4,  align 4
    uint32_t height;            // size 4,  align 4
    int32_t  pitch;             // size 4,  align 4
};
std::optional<Bitmap> get_bitmap(FT_Face const face, char const c) {
    /* load glyph image into the slot (erase previous one) */
    if (FT_Load_Char(face, c, FT_LOAD_RENDER))
        return std::nullopt;  /* ignore errors */

    auto const bitmap_width  = face->glyph->bitmap.width;
    auto const bitmap_pitch = face->glyph->bitmap.pitch;
    auto const bitmap_height = face->glyph->bitmap.rows;
    auto const bitmap_buffer = face->glyph->bitmap.buffer;
#ifdef DEBUG_FT
    for (int i=0; i<bitmap_height; i++) {
        for (int j=0; j<bitmap_width; j++) {
            putchar(bitmap_buffer[i*bitmap_pitch+j] >= 128 ? '#' : ' ');
        }
        puts("");
    }
    puts("'''''''''''''");
#endif
    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);       // GL_TEXTURE_2D is a texture slot the GL provides for texture manipulation
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, static_cast<GLsizei>(bitmap_width), static_cast<GLsizei>(bitmap_height), 0, GL_RED, GL_UNSIGNED_BYTE, bitmap_buffer);  // the texture is uploaded
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);    // here, the properties of said texture are configured
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

#ifdef DEBUG_FT
    {
        auto buffer = static_cast<uint8_t*>(malloc(bitmap_width * bitmap_height));
        glFinish();
        glReadnPixels(0, 0, bitmap_width, bitmap_height, GL_RED, GL_UNSIGNED_BYTE, bitmap_width * bitmap_height, buffer);
        glFinish();
        for (int i=0; i < bitmap_height; i++) {
            for (int j=0; j < bitmap_width; j++) {
                putchar(buffer[i * bitmap_pitch + j] >= 1 ? '#' : ' ');
            }
            puts("");
        }
        free(buffer);
        puts("'''''''''''''");
    }
#endif
    return std::make_optional(Bitmap {
                                      .texture = texture,
                                      .width = bitmap_width,
                                      .height = bitmap_height,
                                      .pitch = bitmap_pitch,
                              });
}

template<int N>
struct Charset {
    std::optional<Bitmap> bitmap[N];
};
template<int N, int M>
Charset<M-N> load_charset(FT_Face const face) {
    Charset<M-N> output;
    for (int i=N; i<M; i++)
        output.bitmap[i-N] = get_bitmap(face, i);
    return output;
}
