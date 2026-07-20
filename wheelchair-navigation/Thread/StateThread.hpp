#pragma once
#include <thread>
#include <atomic>

class StateThread {
public:
    void init();   
    void stop();   
    
private:
    std::atomic<bool> is_running_{false};
    std::thread main_thread_;
    
    void stateMainLoop(); 
};