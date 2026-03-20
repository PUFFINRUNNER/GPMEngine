#ifndef G_ALLEGRO_TEXTURE_HPP
#define G_ALLEGRO_TEXTURE_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <allegro5/allegro.h>

#include "G.hpp"

namespace G {

    class AllegroTexture2D final : public Texture2D {
    public:
        AllegroTexture2D() = default;
        explicit AllegroTexture2D(ALLEGRO_BITMAP* bitmap, bool takeOwnership = true) noexcept;
        ~AllegroTexture2D() override;

        AllegroTexture2D(const AllegroTexture2D&) = delete;
        AllegroTexture2D& operator=(const AllegroTexture2D&) = delete;

        AllegroTexture2D(AllegroTexture2D&& other) noexcept;
        AllegroTexture2D& operator=(AllegroTexture2D&& other) noexcept;

        [[nodiscard]] static std::shared_ptr<AllegroTexture2D> loadFromFile(const std::string& path);

        [[nodiscard]] int width() const noexcept override;
        [[nodiscard]] int height() const noexcept override;
        [[nodiscard]] bool valid() const noexcept override;
        [[nodiscard]] Color sample(const M::Vector2D& uv) const noexcept override;

        [[nodiscard]] ALLEGRO_BITMAP* bitmap() const noexcept;

    private:
        ALLEGRO_BITMAP* m_bitmap = nullptr;
        bool m_ownsBitmap = true;

        int m_width = 0;
        int m_height = 0;
        std::vector<Color> m_pixels;

        void cachePixels();
    };

} // namespace G

#endif