#pragma once

class ReentryGuard {
    public:
        explicit ReentryGuard(bool& entered) : entered(entered), acquired(!entered)
        {
            if (acquired) entered = true;
        }

        ~ReentryGuard()
        {
            if (acquired) entered = false;
        }

        explicit operator bool() const { return acquired; }

        ReentryGuard(const ReentryGuard&) = delete;
        ReentryGuard& operator=(const ReentryGuard&) = delete;

    private:
        bool& entered;
        bool acquired;
};
