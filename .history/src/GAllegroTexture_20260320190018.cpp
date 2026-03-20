#include "GAllegroTexture.hpp"

#include <utility>

#include <allegro5/allegro_image.h>
#include <allegro5/color.h>

namespace {
    [[nodiscard]] float wrap01(float x) noexcept {
        float r = x - std::floor(x);
        if (r < 0.0f) {
            r += 1.0f;
        }
        return r;
    }
}

namespace G {

    AllegroTexture2D::AllegroTexture2D(ALLEGRO_BITMAP* bitmap, bool takeOwnership) noexcept
        : m_bitmap(bitmap), m_ownsBitmap(takeOwnership) {
    }

    AllegroTexture2D::~AllegroTexture2D() {
        if (m_bitmap && m_ownsBitmap) {
            al_destroy_bitmap(m_bitmap);
            m_bitmap = nullptr;
        }
    }

    AllegroTexture2D::AllegroTexture2D(AllegroTexture2D&& other) noexcept
        : m_bitmap(other.m_bitmap), m_ownsBitmap(other.m_ownsBitmap) {
        other.m_bitmap = nullptr;
        other.m_ownsBitmap = true;
    }

    AllegroTexture2D& AllegroTexture2D::operator=(AllegroTexture2D&& other) noexcept {
        if (this == &other) {
            return *this;
        }

        if (m_bitmap && m_ownsBitmap) {
            al_destroy_bitmap(m_bitmap);
        }

        m_bitmap = other.m_bitmap;
        m_ownsBitmap = other.m_ownsBitmap;

        other.m_bitmap = nullptr;
        other.m_ownsBitmap = true;
        return *this;
    }

    std::shared_ptr<AllegroTexture2D> AllegroTexture2D::loadFromFile(const std::string& path) {
        ALLEGRO_BITMAP* bmp = al_load_bitmap(path.c_str());
        if (!bmp) {
            return {};
        }
        return std::make_shared<AllegroTexture2D>(bmp, true);
    }

    int AllegroTexture2D::width() const noexcept {
        return m_bitmap ? al_get_bitmap_width(m_bitmap) : 0;
    }

    int AllegroTexture2D::height() const noexcept {
        return m_bitmap ? al_get_bitmap_height(m_bitmap) : 0;
    }

    bool AllegroTexture2D::valid() const noexcept {
        return m_bitmap != nullptr;
    }

    Color AllegroTexture2D::sample(const M::Vector2D& uv) const noexcept {
        if (!m_bitmap) {
            return Color::white();
        }

        const float u = wrap01(uv.v);
        const float v = wrap01(uv.w);

        const int w = width();
        const int h = height();
        if (w <= 0 || h <= 0) {
            return Color::white();
        }

        const int x = M::clamp(static_cast<int>(u * static_cast<float>(w - 1)), 0, w - 1);
        const int y = std::clamp(static_cast<int>(v * static_cast<float>(h - 1)), 0, h - 1);

        float r = 1.0f;
        float g = 1.0f;
        float b = 1.0f;
        float a = 1.0f;
        al_unmap_rgba_f(al_get_pixel(m_bitmap, x, y), &r, &g, &b, &a);

        return Color(r, g, b, a);
    }

    ALLEGRO_BITMAP* AllegroTexture2D::bitmap() const noexcept {
        return m_bitmap;
    }

} // namespace G