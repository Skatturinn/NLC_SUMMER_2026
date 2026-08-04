#include <iostream>
#include <thread>
#include <chrono>
using namespace std;

void test(int& a) {
	while (a < 10) {
		std::cout << a << std::endl;
		std::this_thread::sleep_for(500ms);
		a++;
	};
	return;
};

int main() {
	int b = 1;

	std::thread t1(test, std::ref(b));
	t1.detach();
	while (b < 5) {
		std::cout << 0 << std::endl;
		std::this_thread::sleep_for(250ms);
	}
	std::this_thread::sleep_for(2000ms);
	return 0;
}
