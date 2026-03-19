#include "Run.hpp"

#include <stdexcept>
#include <sstream>
#include <algorithm>

#include <allegro5/allegro.h>
#include <allegro5/allegro_primitives.h>
#include <allegro5/allegro_font.h>
#include <allegro5/allegro_ttf.h>

namespace Run {

    void App::onInit() {}
    void App::onShutdown() {}
    void App::onResize(int, int) {}
    void App::onFocusGained() {}
    void App::onFocusLost() {}
    void App::onFixedUpdate(double) {}
    void App::onUpdate(double) {}
    void App::onRender(const G::Viewport&, double) {}
    void App::onKeyDown(int) {}
    void App::onKeyUp(int) {}
    void App::onMouseButtonDown(int, float, float) {}
    void App::onMouseButtonUp(int, float, float) {}
    void App::onMouseMove(float, float, float, float) {}

    Runtime::Runtime(Config cfg)
        : m_cfg(std::move(cfg)) {}

    Runtime::~Runtime() {
        shutdown();
    }

    void Runtime::run(App& app) {
        initAllegro();
        createDisplay();
        createTimerAndQueue();

        app.onInit();
        app.onResize(m_viewport.width, m_viewport.height);

        al_start_timer(m_timer);

        bool running = true;
        bool redrawRequested = true;
        bool fullscreen = false;

        double previousTime = al_get_time();
        double accumulator = 0.0;
        double fpsTimer = 0.0;
        int fpsFrames = 0;
        double fpsCurrent = 0.0;

        while (running) {
            ALLEGRO_EVENT ev;
            al_wait_for_event(m_queue, &ev);

            switch (ev.type) {
                case ALLEGRO_EVENT_DISPLAY_CLOSE:
                    running = false;
                    break;

                case ALLEGRO_EVENT_DISPLAY_RESIZE:
                    al_acknowledge_resize(m_display);
                    updateViewportFromDisplay();
                    app.onResize(m_viewport.width, m_viewport.height);
                    redrawRequested = true;
                    break;

                case ALLEGRO_EVENT_DISPLAY_SWITCH_IN:
                    app.onFocusGained();
                    break;

                case ALLEGRO_EVENT_DISPLAY_SWITCH_OUT:
                    app.onFocusLost();
                    break;

                case ALLEGRO_EVENT_TIMER: {
                    const double now = al_get_time();
                    double frameDt = now - previousTime;
                    previousTime = now;
                    frameDt = std::min(frameDt, m_cfg.maxFrameDt);
                    accumulator += frameDt;
                    fpsTimer += frameDt;
                    ++fpsFrames;

                    const double fixedDt = 1.0 / std::max(1.0, m_cfg.fixedPhysicsHz);
                    while (accumulator >= fixedDt) {
                        app.onFixedUpdate(fixedDt);
                        accumulator -= fixedDt;
                    }

                    app.onUpdate(frameDt);
                    redrawRequested = true;

                    if (fpsTimer >= 1.0) {
                        fpsCurrent = static_cast<double>(fpsFrames) / fpsTimer;
                        fpsFrames = 0;
                        fpsTimer = 0.0;
                        if (m_cfg.showFpsInTitle) {
                            updateWindowTitle(fpsCurrent);
                        }
                    }
                    break;
                }

                case ALLEGRO_EVENT_KEY_DOWN:
                    if (m_cfg.allowFullscreenToggle && ev.keyboard.keycode == ALLEGRO_KEY_F11) {
                        fullscreen = !fullscreen;
                        toggleFullscreen(fullscreen);
                        updateViewportFromDisplay();
                        app.onResize(m_viewport.width, m_viewport.height);
                        redrawRequested = true;
                    }
                    app.onKeyDown(ev.keyboard.keycode);
                    break;

                case ALLEGRO_EVENT_KEY_UP:
                    app.onKeyUp(ev.keyboard.keycode);
                    break;

                case ALLEGRO_EVENT_MOUSE_BUTTON_DOWN:
                    app.onMouseButtonDown(ev.mouse.button, ev.mouse.x, ev.mouse.y);
                    break;

                case ALLEGRO_EVENT_MOUSE_BUTTON_UP:
                    app.onMouseButtonUp(ev.mouse.button, ev.mouse.x, ev.mouse.y);
                    break;

                case ALLEGRO_EVENT_MOUSE_AXES:
                    app.onMouseMove(
                        static_cast<float>(ev.mouse.x),
                        static_cast<float>(ev.mouse.y),
                        static_cast<float>(ev.mouse.dx),
                        static_cast<float>(ev.mouse.dy)
                    );
                    break;

                default:
                    break;
            }

            if (redrawRequested && al_is_event_queue_empty(m_queue)) {
                const double alpha = accumulator / std::max(1.0 / std::max(1.0, m_cfg.fixedPhysicsHz), 1e-9);
                beginFrame();
                app.onRender(m_viewport, alpha);
                drawOverlay(fpsCurrent);
                al_flip_display();
                redrawRequested = false;
            }
        }

        app.onShutdown();
    }

    const G::Viewport& Runtime::viewport() const noexcept {
        return m_viewport;
    }

    ALLEGRO_DISPLAY* Runtime::display() const noexcept {
        return m_display;
    }

    const Config& Runtime::config() const noexcept {
        return m_cfg;
    }

    void Runtime::initAllegro() {
        if (!al_init()) {
            throw std::runtime_error("Failed to initialize Allegro.");
        }
        if (!al_install_keyboard()) {
            throw std::runtime_error("Failed to initialize keyboard.");
        }
        if (!al_install_mouse()) {
            throw std::runtime_error("Failed to initialize mouse.");
        }
        if (!al_init_primitives_addon()) {
            throw std::runtime_error("Failed to initialize primitives addon.");
        }
        if (!al_init_font_addon()) {
            throw std::runtime_error("Failed to initialize font addon.");
        }
        if (!al_init_ttf_addon()) {
            throw std::runtime_error("Failed to initialize TTF addon.");
        }

        m_font = al_create_builtin_font();
        if (!m_font) {
            throw std::runtime_error("Failed to create builtin font.");
        }
    }

    void Runtime::createDisplay() {
        if (m_cfg.vsync) {
            al_set_new_display_option(ALLEGRO_VSYNC, 1, ALLEGRO_SUGGEST);
        } else {
            al_set_new_display_option(ALLEGRO_VSYNC, 2, ALLEGRO_SUGGEST);
        }

        if (m_cfg.displayFlags == 0) {
            m_cfg.displayFlags = ALLEGRO_WINDOWED | ALLEGRO_RESIZABLE;
        }

        al_set_new_display_flags(m_cfg.displayFlags);
        m_display = al_create_display(m_cfg.startupResolution.width, m_cfg.startupResolution.height);
        if (!m_display) {
            throw std::runtime_error("Failed to create display.");
        }

        al_set_window_title(m_display, m_cfg.title.c_str());
        updateViewportFromDisplay();
    }

    void Runtime::createTimerAndQueue() {
        const double fps = std::max(1.0, m_cfg.targetFps);
        m_timer = al_create_timer(1.0 / fps);
        if (!m_timer) {
            throw std::runtime_error("Failed to create timer.");
        }

        m_queue = al_create_event_queue();
        if (!m_queue) {
            throw std::runtime_error("Failed to create event queue.");
        }

        al_register_event_source(m_queue, al_get_display_event_source(m_display));
        al_register_event_source(m_queue, al_get_timer_event_source(m_timer));
        al_register_event_source(m_queue, al_get_keyboard_event_source());
        al_register_event_source(m_queue, al_get_mouse_event_source());
    }

    void Runtime::updateViewportFromDisplay() noexcept {
        if (!m_display) return;
        m_viewport.x = 0;
        m_viewport.y = 0;
        m_viewport.width = al_get_display_width(m_display);
        m_viewport.height = al_get_display_height(m_display);
    }

    void Runtime::toggleFullscreen(bool enabled) {
        if (!m_display) return;
        const int ok = al_set_display_flag(m_display, ALLEGRO_FULLSCREEN_WINDOW, enabled);
        if (!ok) {
            al_set_display_flag(m_display, ALLEGRO_FULLSCREEN_WINDOW, false);
        }
    }

    void Runtime::beginFrame() {
        if (m_cfg.clearOnRender) {
            al_clear_to_color(al_map_rgba_f(
                m_cfg.clearColor.r,
                m_cfg.clearColor.g,
                m_cfg.clearColor.b,
                m_cfg.clearColor.a
            ));
        }
    }

    void Runtime::drawOverlay(double fpsCurrent) {
        if (!m_font) return;

        std::ostringstream ss;
        ss << "Resolution: " << m_viewport.width << 'x' << m_viewport.height
           << " | Target FPS: " << static_cast<int>(m_cfg.targetFps)
           << " | FPS: " << static_cast<int>(fpsCurrent + 0.5);

        al_draw_text(
            m_font,
            al_map_rgb(255, 255, 255),
            8.0f,
            8.0f,
            0,
            ss.str().c_str()
        );
    }

    void Runtime::updateWindowTitle(double fpsCurrent) {
        if (!m_display) return;
        std::ostringstream ss;
        ss << m_cfg.title << " | "
           << m_viewport.width << 'x' << m_viewport.height
           << " | FPS " << static_cast<int>(fpsCurrent + 0.5);
        al_set_window_title(m_display, ss.str().c_str());
    }

    void Runtime::shutdown() noexcept {
        if (m_timer) {
            al_destroy_timer(m_timer);
            m_timer = nullptr;
        }
        if (m_queue) {
            al_destroy_event_queue(m_queue);
            m_queue = nullptr;
        }
        if (m_font) {
            al_destroy_font(m_font);
            m_font = nullptr;
        }
        if (m_display) {
            al_destroy_display(m_display);
            m_display = nullptr;
        }
        al_shutdown_ttf_addon();
        al_shutdown_font_addon();
        al_shutdown_primitives_addon();
        al_uninstall_mouse();
        al_uninstall_keyboard();
    }


    
} // namespace Run
