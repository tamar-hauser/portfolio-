#include "FrameDatabase.hpp"
#include <iostream>
#include <stdexcept>

// ──────────────────────────────────────────────────────────────
// Singleton
// ──────────────────────────────────────────────────────────────
FrameDatabase& FrameDatabase::getInstance() {
    static FrameDatabase instance;
    return instance;
}

FrameDatabase::~FrameDatabase() {
    if (db_) sqlite3_close(db_);
}

// ──────────────────────────────────────────────────────────────
// open
// ──────────────────────────────────────────────────────────────
void FrameDatabase::open(const std::string& path) {
    if (db_) return;

    int rc = sqlite3_open(path.c_str(), &db_);
    if (rc != SQLITE_OK) {
        std::cerr << "[DB] Failed to open: " << sqlite3_errmsg(db_) << std::endl;
        sqlite3_close(db_);
        db_ = nullptr;
        return;
    }

    // WAL mode: faster concurrent writes, readers never block writers
    exec("PRAGMA journal_mode=WAL;");
    exec("PRAGMA synchronous=NORMAL;");
    createTables();

    std::cout << "[DB] Opened sensor frame database: " << path << std::endl;
}

// ──────────────────────────────────────────────────────────────
// exec — fire-and-forget helper for DDL / PRAGMA
// ──────────────────────────────────────────────────────────────
void FrameDatabase::exec(const char* sql) {
    char* err = nullptr;
    if (sqlite3_exec(db_, sql, nullptr, nullptr, &err) != SQLITE_OK) {
        std::cerr << "[DB] SQL error: " << (err ? err : "unknown") << std::endl;
        sqlite3_free(err);
    }
}

// ──────────────────────────────────────────────────────────────
// createTables
// ──────────────────────────────────────────────────────────────
void FrameDatabase::createTables() {
    // One row per sensor frame — GPS + per-sensor summary counts + averages
    exec(R"(
        CREATE TABLE IF NOT EXISTS frames (
            id              INTEGER PRIMARY KEY AUTOINCREMENT,
            frame_bucket    INTEGER NOT NULL UNIQUE,
            timestamp       REAL    NOT NULL,
            is_processed    INTEGER NOT NULL DEFAULT 0,

            -- GPS (x_local = latitude, y_local = longitude after GpsProcessing)
            gps_valid       INTEGER,
            gps_latitude    REAL,
            gps_longitude   REAL,
            gps_altitude    REAL,
            gps_speed       REAL,
            gps_heading     REAL,
            gps_confidence  REAL,

            -- IMU (average over all samples in the frame)
            imu_count       INTEGER DEFAULT 0,
            imu_avg_pitch   REAL,
            imu_avg_yaw     REAL,
            imu_avg_vyaw    REAL,
            imu_avg_ax      REAL,

            -- Encoder (average over all samples in the frame)
            enc_count       INTEGER DEFAULT 0,
            enc_avg_v_lin   REAL,
            enc_avg_v_ang   REAL,

            -- Object counts (detail rows in child tables)
            lidar_count     INTEGER DEFAULT 0,
            radar_count     INTEGER DEFAULT 0,
            camera_count    INTEGER DEFAULT 0
        );
    )");

    exec(R"(
        CREATE TABLE IF NOT EXISTS lidar_objects (
            id           INTEGER PRIMARY KEY AUTOINCREMENT,
            frame_bucket INTEGER NOT NULL,
            obj_index    INTEGER NOT NULL,
            pos_x        REAL, pos_y REAL, pos_z REAL,
            yaw          REAL,
            length       REAL, width REAL, height REAL,
            confidence   REAL,
            FOREIGN KEY (frame_bucket) REFERENCES frames(frame_bucket)
        );
    )");

    exec(R"(
        CREATE TABLE IF NOT EXISTS radar_objects (
            id           INTEGER PRIMARY KEY AUTOINCREMENT,
            frame_bucket INTEGER NOT NULL,
            obj_index    INTEGER NOT NULL,
            range_m      REAL,
            pos_x        REAL, pos_y REAL, pos_z REAL,
            vel_x        REAL, vel_y REAL, vel_z REAL,
            rcs          REAL,
            confidence   REAL,
            FOREIGN KEY (frame_bucket) REFERENCES frames(frame_bucket)
        );
    )");

    exec(R"(
        CREATE TABLE IF NOT EXISTS camera_objects (
            id           INTEGER PRIMARY KEY AUTOINCREMENT,
            frame_bucket INTEGER NOT NULL,
            obj_index    INTEGER NOT NULL,
            type_label   TEXT,
            pos_x        REAL, pos_y REAL, pos_z REAL,
            length       REAL, width REAL, height REAL,
            confidence   REAL,
            FOREIGN KEY (frame_bucket) REFERENCES frames(frame_bucket)
        );
    )");

    // Indexes to speed up management-app queries (range lookups by time)
    exec("CREATE INDEX IF NOT EXISTS idx_frames_ts     ON frames(timestamp);");
    exec("CREATE INDEX IF NOT EXISTS idx_lidar_bucket  ON lidar_objects(frame_bucket);");
    exec("CREATE INDEX IF NOT EXISTS idx_radar_bucket  ON radar_objects(frame_bucket);");
    exec("CREATE INDEX IF NOT EXISTS idx_camera_bucket ON camera_objects(frame_bucket);");
}

// ──────────────────────────────────────────────────────────────
// saveFrame
// ──────────────────────────────────────────────────────────────
void FrameDatabase::saveFrame(long long bucket, const FramePointers& frame) {
    // Auto-open with default path so callers don't have to call open() explicitly
    if (!db_) open();
    if (!db_) return;

    // ── GPS ────────────────────────────────────────────────────
    int    gps_valid   = 0;
    double gps_lat = 0, gps_lon = 0, gps_alt = 0;
    double gps_speed = 0, gps_heading = 0, gps_conf = 0;

    if (frame.gps) {
        gps_valid   = frame.gps->isValid ? 1 : 0;
        // After GpsProcessing::createObject: x_local=latitude, y_local=longitude
        gps_lat     = frame.gps->x_local;
        gps_lon     = frame.gps->y_local;
        gps_alt     = frame.gps->z_local;
        gps_speed   = frame.gps->speed;
        gps_heading = frame.gps->heading;
        gps_conf    = frame.gps->confidence;
    }

    // ── IMU (average over all samples) ─────────────────────────
    int    imu_count = 0;
    double imu_pitch = 0, imu_yaw = 0, imu_vyaw = 0, imu_ax = 0;

    if (frame.imu && !frame.imu->list.empty()) {
        imu_count = static_cast<int>(frame.imu->list.size());
        for (const auto& s : frame.imu->list) {
            imu_pitch += s.Pitch;
            imu_yaw   += s.Yaw;
            imu_vyaw  += s.Vyaw;
            imu_ax    += s.Ax;
        }
        imu_pitch /= imu_count;
        imu_yaw   /= imu_count;
        imu_vyaw  /= imu_count;
        imu_ax    /= imu_count;
    }

    // ── Encoder (average over all samples) ─────────────────────
    int    enc_count = 0;
    double enc_vlin = 0, enc_vang = 0;

    if (frame.encoder && !frame.encoder->list.empty()) {
        enc_count = static_cast<int>(frame.encoder->list.size());
        for (const auto& s : frame.encoder->list) {
            enc_vlin += s.v_linear;
            enc_vang += s.v_angular;
        }
        enc_vlin /= enc_count;
        enc_vang /= enc_count;
    }

    const int lidar_count  = frame.lidar  ? static_cast<int>(frame.lidar->list.size())  : 0;
    const int radar_count  = frame.radar  ? static_cast<int>(frame.radar->list.size())  : 0;
    const int camera_count = frame.camera ? static_cast<int>(frame.camera->list.size()) : 0;

    // ── Wrap everything in one transaction for performance ──────
    exec("BEGIN TRANSACTION;");

    // ── Insert main frame row ───────────────────────────────────
    {
        const char* sql = R"(
            INSERT OR IGNORE INTO frames
                (frame_bucket, timestamp, is_processed,
                 gps_valid, gps_latitude, gps_longitude, gps_altitude,
                 gps_speed, gps_heading, gps_confidence,
                 imu_count, imu_avg_pitch, imu_avg_yaw, imu_avg_vyaw, imu_avg_ax,
                 enc_count, enc_avg_v_lin, enc_avg_v_ang,
                 lidar_count, radar_count, camera_count)
            VALUES (?,?,?,  ?,?,?,?,?,?,?,  ?,?,?,?,?,  ?,?,?,  ?,?,?);
        )";
        sqlite3_stmt* stmt = nullptr;
        if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
            std::cerr << "[DB] prepare frames: " << sqlite3_errmsg(db_) << std::endl;
            exec("ROLLBACK;");
            return;
        }
        int i = 1;
        sqlite3_bind_int64 (stmt, i++, bucket);
        sqlite3_bind_double(stmt, i++, frame.timestamp);
        sqlite3_bind_int   (stmt, i++, frame.is_processed ? 1 : 0);
        sqlite3_bind_int   (stmt, i++, gps_valid);
        sqlite3_bind_double(stmt, i++, gps_lat);
        sqlite3_bind_double(stmt, i++, gps_lon);
        sqlite3_bind_double(stmt, i++, gps_alt);
        sqlite3_bind_double(stmt, i++, gps_speed);
        sqlite3_bind_double(stmt, i++, gps_heading);
        sqlite3_bind_double(stmt, i++, gps_conf);
        sqlite3_bind_int   (stmt, i++, imu_count);
        sqlite3_bind_double(stmt, i++, imu_pitch);
        sqlite3_bind_double(stmt, i++, imu_yaw);
        sqlite3_bind_double(stmt, i++, imu_vyaw);
        sqlite3_bind_double(stmt, i++, imu_ax);
        sqlite3_bind_int   (stmt, i++, enc_count);
        sqlite3_bind_double(stmt, i++, enc_vlin);
        sqlite3_bind_double(stmt, i++, enc_vang);
        sqlite3_bind_int   (stmt, i++, lidar_count);
        sqlite3_bind_int   (stmt, i++, radar_count);
        sqlite3_bind_int   (stmt, i++, camera_count);
        sqlite3_step(stmt);
        sqlite3_finalize(stmt);
    }

    // ── Lidar objects ───────────────────────────────────────────
    if (frame.lidar && lidar_count > 0) {
        const char* sql = R"(
            INSERT INTO lidar_objects
                (frame_bucket, obj_index,
                 pos_x, pos_y, pos_z, yaw,
                 length, width, height, confidence)
            VALUES (?,?,  ?,?,?,?,  ?,?,?,?);
        )";
        sqlite3_stmt* stmt = nullptr;
        sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
        int idx = 0;
        for (const auto& o : frame.lidar->list) {
            sqlite3_reset(stmt);
            sqlite3_bind_int64 (stmt, 1, bucket);
            sqlite3_bind_int   (stmt, 2, idx++);
            sqlite3_bind_double(stmt, 3, o.position.x());
            sqlite3_bind_double(stmt, 4, o.position.y());
            sqlite3_bind_double(stmt, 5, o.position.z());
            sqlite3_bind_double(stmt, 6, o.yaw);
            sqlite3_bind_double(stmt, 7, o.length);
            sqlite3_bind_double(stmt, 8, o.width);
            sqlite3_bind_double(stmt, 9, o.height);
            sqlite3_bind_double(stmt, 10, o.confidence);
            sqlite3_step(stmt);
        }
        sqlite3_finalize(stmt);
    }

    // ── Radar objects ───────────────────────────────────────────
    if (frame.radar && radar_count > 0) {
        const char* sql = R"(
            INSERT INTO radar_objects
                (frame_bucket, obj_index,
                 range_m,
                 pos_x, pos_y, pos_z,
                 vel_x, vel_y, vel_z,
                 rcs, confidence)
            VALUES (?,?,  ?,  ?,?,?,  ?,?,?,  ?,?);
        )";
        sqlite3_stmt* stmt = nullptr;
        sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
        int idx = 0;
        for (const auto& o : frame.radar->list) {
            sqlite3_reset(stmt);
            sqlite3_bind_int64 (stmt, 1,  bucket);
            sqlite3_bind_int   (stmt, 2,  idx++);
            sqlite3_bind_double(stmt, 3,  o.range);
            sqlite3_bind_double(stmt, 4,  o.position.x());
            sqlite3_bind_double(stmt, 5,  o.position.y());
            sqlite3_bind_double(stmt, 6,  o.position.z());
            sqlite3_bind_double(stmt, 7,  o.velocity.x());
            sqlite3_bind_double(stmt, 8,  o.velocity.y());
            sqlite3_bind_double(stmt, 9,  o.velocity.z());
            sqlite3_bind_double(stmt, 10, o.rcs);
            sqlite3_bind_double(stmt, 11, o.confidence);
            sqlite3_step(stmt);
        }
        sqlite3_finalize(stmt);
    }

    // ── Camera objects ──────────────────────────────────────────
    if (frame.camera && camera_count > 0) {
        const char* sql = R"(
            INSERT INTO camera_objects
                (frame_bucket, obj_index, type_label,
                 pos_x, pos_y, pos_z,
                 length, width, height, confidence)
            VALUES (?,?,?,  ?,?,?,  ?,?,?,?);
        )";
        sqlite3_stmt* stmt = nullptr;
        sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
        int idx = 0;
        for (const auto& o : frame.camera->list) {
            sqlite3_reset(stmt);
            sqlite3_bind_int64 (stmt, 1,  bucket);
            sqlite3_bind_int   (stmt, 2,  idx++);
            sqlite3_bind_text  (stmt, 3,  o.type_label.c_str(), -1, SQLITE_TRANSIENT);
            sqlite3_bind_double(stmt, 4,  o.position_3d.x());
            sqlite3_bind_double(stmt, 5,  o.position_3d.y());
            sqlite3_bind_double(stmt, 6,  o.position_3d.z());
            sqlite3_bind_double(stmt, 7,  o.length);
            sqlite3_bind_double(stmt, 8,  o.width);
            sqlite3_bind_double(stmt, 9,  o.height);
            sqlite3_bind_double(stmt, 10, o.confidence);
            sqlite3_step(stmt);
        }
        sqlite3_finalize(stmt);
    }

    exec("COMMIT;");
}
