#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>
#include <string>
#include <utility> // עבור std::move
#include "Constants.hpp" 
#include "KalmanFilter\ekf.hpp"

enum class UpdatePriority {
    LOW = 0,         // עדכונים שגרתיים (למשל, עדכון סטטיסטיקה או לוגים)
    MEDIUM = 1,      // עדכוני חיישנים רגילים (כמו GPS או מצלמה)
    HIGH = 2,        // עדכוני חיישנים מהירים (כמו IMU או קורא אנקודרים)
    CRITICAL = 3     // עדכוני חירום (למשל, זיהוי התנגשות מיידית או פקודת עצירה)
};

struct StateUpdateTask {
    UpdatePriority priority;
    double timestamp;
    std::string source = "UNKNOWN";  // לאבחון בלבד: מאיזה צרכן הגיעה המשימה (IMU/ENCODER/...)

    // שינוי: הפונקציה מקבלת כעת רפרנס ישיר לאובייקט ה-EKF של מערכת הניווט שלך
    std::function<void(EKF<Config::DimWheelchairStateVector>& Statevector)> execute;

    // timestamp הוא הקריטריון הראשי לשליפה (כרונולוגי, הישן ביותר ראשון) —
    // priority משמש רק כ-tie-break כששני timestamps שווים. זה הכרחי כדי ש-
    // StateThread יוכל לחשב dt תקין ל-predict (ראה [EKF_DT_DEBUG]).
    bool operator<(const StateUpdateTask& other) const {
        if (this->timestamp != other.timestamp) {
            return this->timestamp > other.timestamp;
        }
        return this->priority < other.priority;
    }
};

template <typename T>
class ThreadSafePriorityQueue {
private:
    std::priority_queue<T> pq; // תור עדיפויות מובנה של C++
    std::mutex mtx;
    std::condition_variable cv;

public:
    // דחיפת משימה לתור
    void push(T task) {
        std::unique_lock<std::mutex> lock(mtx);
        pq.push(std::move(task)); // תוקן: שימוש ב-move לייעול
        cv.notify_one(); 
    }

    // שליפת המשימה בעלת העדיפות הגבוהה ביותר (חוסם אם התור ריק)
    T pop() {
        std::unique_lock<std::mutex> lock(mtx);
        // תוקן: שימוש ב-cv (בלי קו תחתון) כדי להתאים להגדרה ב-private
        cv.wait(lock, [this]() { return !pq.empty(); });
        
        T top_task = std::move(const_cast<T&>(pq.top())); // שליפת האלמנט העליון ביעילות
        pq.pop(); // תוקן: שימוש ב-pq (בלי קו תחתון)
        
        return top_task;
    }
};