#include <iostream>
#include <chrono>
#include <thread>
// #include <thread> // For sleep_for

int main() {

    // std::chrono::duration<int, std::ratio<1, 300>> test(1);
    
    // std::chrono::duration<double, std::milli> ms_duration = test;
    
    // std::cout << ms_duration.count() << std::endl;
    // // std::cout << "Elapsed time: " << ms << " ms (" << ns << " ns)\n";
    // auto start = std::chrono::steady_clock::now();
    // auto target_time = start + test;
    bool running_ = true;
    int i = 0;
    
    
    // 1. setup the interval/tick length
    // create a 20hz, 1/20, 50ms 
    const auto interval = std::chrono::microseconds(50);
    
    
    // 2. initialize, current time
    // We run immediately, initialize the time
    auto next_wake_time = std::chrono::steady_clock::now();
    
    while (running_) {
        
        
        // 3. iterate
        
        next_wake_time += interval;
        
        // 4. computation
        
        std::cout << i++ << std::endl;
        
        if (i > 30) {
            running_ = false;
        }
        
        
        // 5. We sleep until we are close to next target
        // setup target as a millisecond before next run.
        auto sleep_target = next_wake_time - std::chrono::milliseconds(1);
        
        
        // sleep
        if (std::chrono::steady_clock::now() < sleep_target) {
            std::this_thread::sleep_until(sleep_target);
        }
        
        // loop until it is go time, "warm up"
        
        while (std::chrono::steady_clock::now() < next_wake_time) {}
    }
    
    
    
    return 0;
}