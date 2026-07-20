#pragma once
#include <thread>
#include <atomic>
#include <vector>
#include <functional>

class SensorWorkerManager {
private:
    std::atomic<bool> is_running{false};
    std::vector<std::thread> workers;

public:
    template<typename Func>
    void startWorker(Func process_logic) {
        is_running = true;
        workers.emplace_back([this, process_logic]() {
            while (is_running) {
                process_logic();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }

    void stop() {
        is_running = false;
        for (auto& t : workers) {
            if (t.joinable()) t.join();
        }
    }
};