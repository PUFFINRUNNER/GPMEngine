#ifndef RUN_HPP
#define RUN_HPP

#include <allegro5/allegro_font.h>
#include <allegro5/allegro_image.h>
#include <allegro5/allegro_primitives.h>
#include <allegro5/allegro_ttf.h>
#include <allegro5/color.h>
#include <allegro5/events.h>
#include <allegro5/keyboard.h>
#include <allegro5/mouse.h>
#include <allegro5/system.h>
#include <allegro5/timer.h>
#include <cstddef>

#define forever while (true)

class Display {
  private:
    ALLEGRO_DISPLAY *display = nullptr;
  public:
    Display(ALLEGRO_DISPLAY *dsp): display(dsp) {}
    void update_stats() {}
    ALLEGRO_DISPLAY
};

class Run {
  private:
    inline static bool initialized = false;
    bool stopped = true;
    ALLEGRO_TIMER *timer = nullptr;
    Display *display = nullptr;
    ALLEGRO_EVENT_QUEUE *event_queue = nullptr;

  protected:
    // Color that fills the screen before `render_process`
    // SEE: `full_redraw` for more info
    ALLEGRO_COLOR bg_color = {0, 0, 0, 0};

    // TRUE: before each `render_process` screen is filled with `bg_color`
    // FALSE: screen remains unchanged before `render_process`
    // NOTE: can be used, to skip `render_process` if there is nothing to render
    bool full_redraw = true;

    // TRUE: outputs debug info into console
    bool verbose = true;

    // Called each frame to update internal logic
    virtual void physics_process() = 0;

    // Called each frame to redraw all things on screen
    virtual void render_process() = 0;

  public:
    static const std::size_t FPS = 60;
    static const std::size_t WIDTH = 640;
    static const std::size_t HEIGHT = 360;

    // Initialize and start the game loop
    void start();

    // Stops game loop
    void stop();

    void update_display_stats(Display *dsp);
};

#endif