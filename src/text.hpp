#pragma once
#include <cstdint>
#include <ft2build.h>
#include FT_FREETYPE_H

// where f is callable
template<typename F>
static inline
void render_char(
    FT_Face const face,
    char* const text,
    std::size_t const num_chars,
    float const left,
    float const top,
    float const size_of_pixel,
    F const draw_bitmap) {

    int pen_x = 0;
    int pen_y = 0;
    FT_GlyphSlot slot = face->glyph;
    for (int n = 0; n < num_chars; n++) {
        FT_Error error;
        FT_UInt glyph_index;
        // character code ~ ASCII ordinal
        // glyph index = index into glyph array, containing render data
        /* retrieve glyph index from character code */
        glyph_index = FT_Get_Char_Index(face, text[n]);

        /* load glyph image into the slot (erase previous one) */
        error = FT_Load_Glyph(face, glyph_index, FT_LOAD_RENDER);
        if ( error )
            continue;  /* ignore errors */

        /* convert to an anti-aliased bitmap */
        error = FT_Render_Glyph(face->glyph, FT_RENDER_MODE_NORMAL);
        if ( error )
            continue;

        /* now, draw to our target surface */
        auto const screen_left = pen_x + slot->bitmap_left;
        auto const screen_top = pen_y;// - slot->bitmap_top;
        auto const width  = slot->bitmap.width * size_of_pixel;
        auto const height = slot->bitmap.rows * size_of_pixel;
        auto const device_left = screen_left * size_of_pixel;
        auto const device_top = screen_top * size_of_pixel;
        auto const device_right = device_left + width;
        auto const device_bottom = device_top - height;
        draw_bitmap(&slot->bitmap,
                    device_left,
                    device_top,
                    device_right,
                    device_bottom);

        /* increment pen position */
        pen_x += slot->advance.x >> 6;
        pen_y += slot->advance.y >> 6; /* not useful for now */
    }
}
