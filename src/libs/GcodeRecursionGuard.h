#pragma once

class GcodeRecursionGuard {
  public:
    explicit GcodeRecursionGuard(bool &handling_gcode)
        : handling_gcode_(handling_gcode), entered_(!handling_gcode)
    {
        if (entered_)
            handling_gcode_ = true;
    }

    ~GcodeRecursionGuard()
    {
        if (entered_)
            handling_gcode_ = false;
    }

    explicit operator bool() const
    {
        return entered_;
    }

    GcodeRecursionGuard(const GcodeRecursionGuard &) = delete;
    GcodeRecursionGuard &operator=(const GcodeRecursionGuard &) = delete;

  private:
    bool &handling_gcode_;
    bool entered_;
};
