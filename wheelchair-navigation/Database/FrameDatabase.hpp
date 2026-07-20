#pragma once
#include <string>
#include <sqlite3.h>
#include "SensorFrame.hpp"

// Singleton that persists FramePointers to an SQLite database before they are
// evicted from SensorFrameManager. Call open() once at startup (optional —
// saveFrame will auto-open with the default path on first use).
class FrameDatabase {
public:
    static FrameDatabase& getInstance();

    // Opens (or creates) the database at 'path'. Safe to call multiple times —
    // subsequent calls are no-ops if the database is already open.
    void open(const std::string& path = "sensor_frames.db");

    // Serialises one frame to the database. Thread-safe relative to other
    // saveFrame calls (SQLite WAL + single connection serialises naturally).
    void saveFrame(long long bucket, const FramePointers& frame);

    ~FrameDatabase();

private:
    FrameDatabase() = default;
    void createTables();
    void exec(const char* sql);

    sqlite3* db_ = nullptr;
};
