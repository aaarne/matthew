#include <iostream>
#include <thread>
#include <chrono>

#ifndef TIMER_CPP
#define TIMER_CPP

class Timer {
    bool clear = false;

    public:
        void setTimeout(std::function<void(void)> function, int delay);
        void setInterval(std::function<void(void)> function, int interval);
        void stop();

};

#endif