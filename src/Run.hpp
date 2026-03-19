#ifndef RUN_HPP
#define RUN_HPP

#include <string>
#include "G.hpp"

struct ALLEGRO_DISPLAY;
struct ALLEGRO_EVENT_QUEUE;
struct ALLEGRO_TIMER;
struct ALLEGRO_FONT;

namespace Run {

    struct Resolution {
        int width = 1280;
        int height = 720;

        constexpr Resolution() noexcept = default;
        constexpr Resolution(int w, int h) noexcept : width(w), height(h) {}
    };

    struct Config {
        std::string title = "Engine";
        Resolution startupResolution {1280, 720};
        int displayFlags = 0;

        bool vsync = true;
        bool showFpsInTitle = true;
        bool allowFullscreenToggle = true;
        bool clearOnRender = true;

        double targetFps = 60.0;
        double fixedPhysicsHz = 60.0;
        double maxFrameDt = 0.25;

        G::Color clearColor = G::Color::black();
    };

    class App {
    public:
        virtual ~App() = default;

        virtual void onInit();
        virtual void onShutdown();

        virtual void onResize(int width, int height);
        virtual void onFocusGained();
        virtual void onFocusLost();

        virtual void onFixedUpdate(double dt);
        virtual void onUpdate(double dt);
        virtual void onRender(const G::Viewport& viewport, double alpha);

        virtual void onKeyDown(int keycode);
        virtual void onKeyUp(int keycode);
        virtual void onMouseButtonDown(int button, float x, float y);
        virtual void onMouseButtonUp(int button, float x, float y);
        virtual void onMouseMove(float x, float y, float dx, float dy);
    };

    class Runtime {
    public:
        explicit Runtime(Config cfg);
        ~Runtime();

        Runtime(const Runtime&) = delete;
        Runtime& operator=(const Runtime&) = delete;

        void run(App& app);

        [[nodiscard]] const G::Viewport& viewport() const noexcept;
        [[nodiscard]] ALLEGRO_DISPLAY* display() const noexcept;
        [[nodiscard]] const Config& config() const noexcept;

    private:
        Config m_cfg;
        ALLEGRO_DISPLAY* m_display = nullptr;
        ALLEGRO_EVENT_QUEUE* m_queue = nullptr;
        ALLEGRO_TIMER* m_timer = nullptr;
        ALLEGRO_FONT* m_font = nullptr;
        G::Viewport m_viewport;

        void initAllegro();
        void createDisplay();
        void createTimerAndQueue();
        void updateViewportFromDisplay() noexcept;
        void toggleFullscreen(bool enabled);
        void beginFrame();
        void drawOverlay(double fpsCurrent);
        void updateWindowTitle(double fpsCurrent);
        void shutdown() noexcept;
    };

} // namespace Run

#endif
