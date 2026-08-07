#pragma once

#include "Module.h"

class Gcode;

class BedCleaning : public Module {
  public:
    BedCleaning();

    void on_module_loaded() override;
    void on_gcode_received(void *argument) override;
    void on_abort(void *argument) override;
    void on_halt(void *argument) override;

  private:
    bool move(float x, float y);
    bool clean(unsigned int cycles, float spacing);
    bool player_is_busy() const;
    void set_player_busy(bool busy) const;
    void finish();

    float x_min_;
    float x_max_;
    float y_min_;
    float y_max_;
    float park_y_;
    float front_x_;
    float spacing_;
    float cycles_;
    bool active_;
};
