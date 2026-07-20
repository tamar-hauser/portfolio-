import express from 'express'
import cors from 'cors'
import { getDb, dbStatus, setDbPath } from './db.js'

const app = express()
const PORT = process.env.PORT || 3001

app.use(cors())
app.use(express.json())

// ── Auth ─────────────────────────────────────────────────────────────────────
// Simple local auth — change credentials here or use env vars
const USERS = {
  [process.env.ADMIN_USER || 'admin']: process.env.ADMIN_PASS || 'wheelchair2024'
}

app.post('/api/auth/login', (req, res) => {
  const { username, password } = req.body || {}
  if (username && USERS[username] === password) {
    res.json({ ok: true, username })
  } else {
    res.status(401).json({ ok: false, error: 'Invalid credentials' })
  }
})

// ── DB status + path config (available without DB) ───────────────────────────
app.get('/api/status', (_req, res) => {
  res.json(dbStatus())
})

app.post('/api/config/db-path', (req, res) => {
  const { path: newPath } = req.body || {}
  if (!newPath) return res.status(400).json({ error: 'path is required' })
  const result = setDbPath(newPath)
  res.json(result)
})

// ── Guard: all data routes below require a live DB connection ─────────────────
app.use('/api', (req, res, next) => {
  const skip = ['/auth/login', '/status', '/config/db-path']
  if (skip.includes(req.path)) return next()
  if (!getDb()) return res.status(503).json({ error: 'Database not available' })
  next()
})

// ── Stats ─────────────────────────────────────────────────────────────────────
app.get('/api/stats', (_req, res) => {
  try {
    const stats = getDb().prepare(`
      SELECT
        COUNT(*)                                           AS total_frames,
        MIN(timestamp)                                     AS first_ts,
        MAX(timestamp)                                     AS last_ts,
        SUM(CASE WHEN gps_valid = 1 THEN 1 ELSE 0 END)    AS gps_valid_count,
        ROUND(AVG(CASE WHEN gps_valid=1 THEN gps_confidence END), 3) AS avg_gps_conf,
        SUM(imu_count)                                     AS total_imu_samples,
        SUM(enc_count)                                     AS total_enc_samples,
        SUM(lidar_count)                                   AS total_lidar_objects,
        SUM(radar_count)                                   AS total_radar_objects,
        SUM(camera_count)                                  AS total_camera_objects
      FROM frames
    `).get()
    res.json(stats)
  } catch (e) {
    res.status(500).json({ error: e.message })
  }
})

// ── Frames list ───────────────────────────────────────────────────────────────
app.get('/api/frames', (req, res) => {
  const { from, to, limit = 200, offset = 0 } = req.query
  let sql = 'SELECT * FROM frames WHERE 1=1'
  const params = []
  if (from) { sql += ' AND timestamp >= ?'; params.push(Number(from)) }
  if (to)   { sql += ' AND timestamp <= ?'; params.push(Number(to))   }
  sql += ' ORDER BY timestamp DESC LIMIT ? OFFSET ?'
  params.push(Number(limit), Number(offset))
  try {
    res.json(getDb().prepare(sql).all(...params))
  } catch (e) {
    res.status(500).json({ error: e.message })
  }
})

// ── Single frame + all sensor children ───────────────────────────────────────
app.get('/api/frames/:bucket', (req, res) => {
  const { bucket } = req.params
  try {
    const db    = getDb()
    const frame = db.prepare('SELECT * FROM frames WHERE frame_bucket = ?').get(bucket)
    if (!frame) return res.status(404).json({ error: 'Frame not found' })

    const lidar  = db.prepare('SELECT * FROM lidar_objects  WHERE frame_bucket = ? ORDER BY obj_index').all(bucket)
    const radar  = db.prepare('SELECT * FROM radar_objects  WHERE frame_bucket = ? ORDER BY obj_index').all(bucket)
    const camera = db.prepare('SELECT * FROM camera_objects WHERE frame_bucket = ? ORDER BY obj_index').all(bucket)

    res.json({ ...frame, lidar, radar, camera })
  } catch (e) {
    res.status(500).json({ error: e.message })
  }
})

// ── GPS trail (for path chart) ────────────────────────────────────────────────
app.get('/api/gps-trail', (req, res) => {
  const { from, to, limit = 500 } = req.query
  let sql = `SELECT frame_bucket, timestamp, gps_latitude AS x, gps_longitude AS y, gps_confidence AS conf
             FROM frames WHERE gps_valid = 1`
  const params = []
  if (from) { sql += ' AND timestamp >= ?'; params.push(Number(from)) }
  if (to)   { sql += ' AND timestamp <= ?'; params.push(Number(to))   }
  sql += ' ORDER BY timestamp ASC LIMIT ?'
  params.push(Number(limit))
  try {
    res.json(getDb().prepare(sql).all(...params))
  } catch (e) {
    res.status(500).json({ error: e.message })
  }
})

// ── Confidence timeline (for sparkline) ──────────────────────────────────────
app.get('/api/confidence-timeline', (req, res) => {
  const { limit = 100 } = req.query
  try {
    const rows = getDb().prepare(`
      SELECT timestamp,
             gps_confidence,
             CASE WHEN imu_count > 0 THEN 1.0 ELSE 0 END AS imu_ok,
             CASE WHEN enc_count > 0 THEN 1.0 ELSE 0 END AS enc_ok
      FROM frames
      ORDER BY timestamp DESC
      LIMIT ?
    `).all(Number(limit))
    res.json(rows.reverse())
  } catch (e) {
    res.status(500).json({ error: e.message })
  }
})

app.listen(PORT, () => {
  console.log(`\n🦽  Wheelchair Admin API  →  http://localhost:${PORT}\n`)
})
