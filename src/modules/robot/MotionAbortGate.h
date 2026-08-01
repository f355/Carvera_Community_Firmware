#pragma once

class MotionAbortGate {
public:
    void request() { requested_ = true; }
    void clear() { requested_ = false; }
    bool active() const { return requested_; }

private:
    volatile bool requested_ = false;
};
