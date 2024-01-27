#pragma once
#include <utility>
#include <optional>
#include <cstdint>
#include <ft2build.h>
#include FT_FREETYPE_H

struct Bitmap {                 // size 32, align 8
    GLuint   texture;  // size 24, align 8
    uint32_t width;             // size 4,  align 2
    uint32_t height;            // size 4,  align 2
};
std::optional<Bitmap> get_bitmap(FT_Face const face, char const c) {
    auto const glyph_index = FT_Get_Char_Index(face, c);
    /* load glyph image into the slot (erase previous one) */
    auto error = FT_Load_Glyph(face, glyph_index, FT_LOAD_RENDER);
    if ( error )
        return std::nullopt;  /* ignore errors */

    /* convert to an anti-aliased bitmap */
    error = FT_Render_Glyph(face->glyph, FT_RENDER_MODE_NORMAL);
    if ( error )
        return std::nullopt;

    auto const bitmap_width  = face->glyph->bitmap.width;
    auto const bitmap_height = face->glyph->bitmap.rows;    
    auto const bitmap_size = bitmap_width * bitmap_height;
    auto const bitmap = face->glyph->bitmap.buffer;
    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);       // GL_TEXTURE_2D is a texture slot the GL provides for texture manipulation
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);    // here, the properties of said texture are configured
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, bitmap_width, bitmap_height, 0, GL_RED, GL_UNSIGNED_BYTE, bitmap);  // the texture is uploaded

    return std::make_optional(Bitmap {
        .texture = texture,
        .width = bitmap_width,
        .height = bitmap_height,
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
