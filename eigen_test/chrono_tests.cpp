#include <iostream>
#include <chrono>

int main()
{
    auto start = std::chrono::steady_clock::now();
    auto run_time = std::chrono::milliseconds(1000);
    auto end_time = start + run_time;
    // std::cout<<"Hello World";
    int a = 0;
    while(std::chrono::steady_clock::now() < end_time) {
        // std::cout << a++ << std::endl;
    }
    auto print = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
    std::cout << print << std::endl;
    return 0;
}