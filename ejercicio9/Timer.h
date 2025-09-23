#ifndef TIMER_H
#define TIMER_H
#include <thread>
#include <chrono>
#include <functional>
#include <future>
#include <cstdio>
#include <iostream>

// Concept to ensure the parameter is callable with no arguments
template<typename T>
concept Callable = std::invocable<T>;

class Timer
{
public:
    Timer(){};
    template <class T>
    void connect(T f)
    {
        std::thread([this, f = std::move(f)]()
        {
            while(true)
            {
                  if(go.load())
                      std::invoke(f);
                  std::this_thread::sleep_for(std::chrono::milliseconds(period.load()));
            }
        }).detach();
    };
    void start(int p)
    {
        go.store(true);
        period.store(p);
    };
    void stop()
    {
        go.store(false);
    };
    void setInterval(int p)
    {
        period.store(p);
    }
private:
    std::atomic_bool go = false;
    std::atomic_int period = 0;
};

#endif // TIMER_H