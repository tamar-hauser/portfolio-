import { fileURLToPath } from 'url'
import path from 'path'
import { DatabaseSync } from 'node:sqlite'  // built-in since Node.js 22.5, stable in Node 24

const __dirname = path.dirname(fileURLToPath(import.meta.url))

const DEFAULT_PATH = path.resolve(
  __dirname, '..', '..', 'controllers', 'wheelchair_cpp_controller', 'sensor_frames.db'
)

let currentPath = process.env.DB_PATH || DEFAULT_PATH
let db = null

export function getDbPath() { return currentPath }

export function setDbPath(newPath) {
  if (db) { try { db.close() } catch {} db = null }
  currentPath = newPath
  return connect()
}

function connect() {
  try {
    db = new DatabaseSync(currentPath, { readOnly: true })
    console.log('[DB] Connected:', currentPath)
    return { connected: true }
  } catch (e) {
    db = null
    console.warn('[DB] Not available:', e.message)
    return { connected: false, error: e.message }
  }
}

export function getDb() {
  if (db) return db
  connect()
  return db
}

export function dbStatus() {
  if (db) return { connected: true, path: currentPath }
  const result = connect()
  return result.connected
    ? { connected: true,  path: currentPath }
    : { connected: false, path: currentPath, error: result.error }
}
