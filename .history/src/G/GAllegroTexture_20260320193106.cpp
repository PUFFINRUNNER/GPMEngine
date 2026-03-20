#include "GAllegroTexture.hpp"

#include <algorithm>
#include <cmath>
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
        cachePixels();
    }

    AllegroTexture2D::~AllegroTexture2D() {
        if (m_bitmap && m_ownsBitmap) {
            al_destroy_bitmap(m_bitmap);
            m_bitmap = nullptr;
        }
    }

    AllegroTexture2D::AllegroTexture2D(AllegroTexture2D&& other) noexcept
        : m_bitmap(other.m_bitmap),
          m_ownsBitmap(other.m_ownsBitmap),
          m_width(other.m_width),
          m_height(other.m_height),
          m_pixels(std::move(other.m_pixels)) {
        other.m_bitmap = nullptr;
        other.m_ownsBitmap = true;
        other.m_width = 0;
        other.m_height = 0;
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
        m_width = other.m_width;
        m_height = other.m_height;
        m_pixels = std::move(other.m_pixels);

        other.m_bitmap = nullptr;
        other.m_ownsBitmap = true;
        other.m_width = 0;
        other.m_height = 0;
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
        return m_width;
    }

    int AllegroTexture2D::height() const noexcept {
        return m_height;
    }

    bool AllegroTexture2D::valid() const noexcept {
        return m_bitmap != nullptr && m_width > 0 && m_height > 0 && !m_pixels.empty();
    }

    void AllegroTexture2D::cachePixels() {
        m_width = 0;
        m_height = 0;
        m_pixels.clear();

        if (!m_bitmap) {
            return;
        }

        m_width = al_get_bitmap_width(m_bitmap);
        m_height = al_get_bitmap_height(m_bitmap);

        if (m_width <= 0 || m_height <= 0) {
            m_width = 0;
            m_height = 0;
            return;
        }

        m_pixels.resize(static_cast<std::size_t>(m_width * m_height));

        ALLEGRO_LOCKED_REGION* lock = al_lock_bitmap(m_bitmap, ALLEGRO_PIXEL_FORMAT_ABGR_8888_LE, ALLEGRO_LOCK_READONLY);
        if (!lock) {
            m_width = 0;
            m_height = 0;
            m_pixels.clear();
            return;
        }

        const std::uint8_t* src = static_cast<const std::uint8_t*>(lock->data);
        for (int y = 0; y < m_height; ++y) {
            const std::uint32_t* row = reinterpret_cast<const std::uint32_t*>(src + y * lock->pitch);
            for (int x = 0; x < m_width; ++x) {
                m_pixels[static_cast<std::size_t>(y * m_width + x)] = row[x];
            }
        }

        al_unlock_bitmap(m_bitmap);
    }

    Color AllegroTexture2D::sample(const M::Vector2D& uv) const noexcept {
        if (!valid()) {
            return Color::white();
        }

        const float u = wrap01(uv.v);
        const float v = wrap01(uv.w);

        const int x = std::clamp(static_cast<int>(u * static_cast<float>(m_width - 1)), 0, m_width - 1);
        const int y = std::clamp(static_cast<int>(v * static_cast<float>(m_height - 1)), 0, m_height - 1);

        const std::uint32_t px = m_pixels[static_cast<std::size_t>(y * m_width + x)];

        const float r = static_cast<float>( px        & 0xFFu) / 255.0f;
        const float g = static_cast<float>((px >> 8)  & 0xFFu) / 255.0f;
        const float b = static_cast<float>((px >> 16) & 0xFFu) / 255.0f;
        const float a = static_cast<float>((px >> 24) & 0xFFu) / 255.0f;

        return Color(r, g, b, a);
    }

    ALLEGRO_BITMAP* AllegroTexture2D::bitmap() const noexcept {
        return m_bitmap;
    }

} // namespace G