import { useState, useEffect, useRef } from 'react'
import { useNavigate } from 'react-router-dom'
import {
  LineChart, Line, XAxis, YAxis, Tooltip, ResponsiveContainer, CartesianGrid,
  ScatterChart, Scatter,
} from 'recharts'
import ConfidenceBar from '../components/ConfidenceBar.jsx'

// ── DB Setup panel (shown when database is not connected) ─────────────────────
function DbSetupPanel({ onConnected }) {
  const [path, setPath]       = useState('')
  const [status, setStatus]   = useState(null)   // null | 'connecting' | 'error'
  const [errMsg, setErrMsg]   = useState('')
  const intervalRef           = useRef(null)

  // Auto-retry every 3 s (e.g. user just started the simulation)
  useEffect(() => {
    intervalRef.current = setInterval(async () => {
      try {
        const s = await fetch('/api/status').then(r => r.json())
        if (s.connected) { clearInterval(intervalRef.current); onConnected() }
      } catch {}
    }, 3000)
    return () => clearInterval(intervalRef.current)
  }, [onConnected])

  async function tryConnect(e) {
    e.preventDefault()
    setStatus('connecting')
    setErrMsg('')
    try {
      const res  = await fetch('/api/config/db-path', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ path }),
      })
      const data = await res.json()
      if (data.connected) { onConnected() }
      else { setStatus('error'); setErrMsg(data.error || 'Could not open database') }
    } catch (err) {
      setStatus('error')
      setErrMsg(err.message)
    }
  }

  return (
    <div style={{ flex: 1, display: 'flex', alignItems: 'center', justifyContent: 'center', padding: 40 }}>
      <div className="card" style={{ maxWidth: 560, width: '100%', padding: '36px 40px' }}>
        {/* Icon */}
        <div style={{ textAlign: 'center', marginBottom: 28 }}>
          <svg width="56" height="56" viewBox="0 0 56 56" style={{ marginBottom: 12 }}>
            <circle cx="28" cy="28" r="24" stroke="#CBE0F0" strokeWidth="2" fill="#EFF6FC" strokeDasharray="6 3"/>
            <circle cx="28" cy="28" r="9" fill="#7BB8D4" opacity="0.4"/>
            <circle cx="28" cy="28" r="5" fill="#2EB89A" opacity="0.6"/>
            <text x="28" y="52" textAnchor="middle" fontSize="11" fill="#95BDD8" fontWeight="700">DB</text>
          </svg>
          <div style={{ fontSize: 19, fontWeight: 700, color: '#1B3967' }}>Database Not Connected</div>
          <div style={{ fontSize: 13, color: '#6B8CAE', marginTop: 6 }}>
            Run the Webots simulation for at least 5 seconds to generate the database file,
            or specify its path below.
          </div>
        </div>

        {/* Default path hint */}
        <div style={{ background: '#F0F6FB', borderRadius: 8, padding: '12px 16px', marginBottom: 20, fontSize: 12 }}>
          <div style={{ fontWeight: 700, color: '#3E72B8', marginBottom: 4 }}>Default location</div>
          <code style={{ color: '#1A2E4A', fontSize: 11, wordBreak: 'break-all' }}>
            controllers/wheelchair_cpp_controller/sensor_frames.db
          </code>
        </div>

        {/* Custom path form */}
        <form onSubmit={tryConnect}>
          <label style={{ display: 'block', fontSize: 11, fontWeight: 700, textTransform: 'uppercase', letterSpacing: '0.7px', color: '#6B8CAE', marginBottom: 6 }}>
            Custom database path (optional)
          </label>
          <div style={{ display: 'flex', gap: 8 }}>
            <input
              style={{ flex: 1, padding: '9px 12px', border: '1.5px solid #CBE0F0', borderRadius: 8, fontSize: 13, outline: 'none', background: '#F8FBFE', color: '#1A2E4A' }}
              placeholder="C:\path\to\sensor_frames.db"
              value={path}
              onChange={e => setPath(e.target.value)}
            />
            <button
              type="submit"
              disabled={!path || status === 'connecting'}
              style={{ padding: '9px 18px', background: 'linear-gradient(135deg, #1B3967, #3E72B8)', color: '#fff', borderRadius: 8, fontWeight: 600, fontSize: 13, opacity: (!path || status === 'connecting') ? 0.6 : 1 }}
            >
              {status === 'connecting' ? '…' : 'Connect'}
            </button>
          </div>
          {status === 'error' && (
            <div style={{ marginTop: 10, color: '#D94F4F', fontSize: 12, background: '#FEF2F2', border: '1px solid #FCA5A5', borderRadius: 6, padding: '8px 12px' }}>
              ⚠ {errMsg}
            </div>
          )}
        </form>

        {/* Auto-retry indicator */}
        <div style={{ marginTop: 20, display: 'flex', alignItems: 'center', gap: 8, color: '#95BDD8', fontSize: 12 }}>
          <span style={{ display: 'inline-block', width: 8, height: 8, borderRadius: '50%', background: '#2EB89A', animation: 'pulse 2s infinite' }} />
          Auto-checking every 3 seconds…
        </div>
      </div>

      <style>{`
        @keyframes pulse {
          0%, 100% { opacity: 1; }
          50%       { opacity: 0.3; }
        }
      `}</style>
    </div>
  )
}

// ── Top nav ──────────────────────────────────────────────────────────────────
function NavBar({ user, onLogout }) {
  return (
    <div style={nav.bar}>
      <div style={nav.logo}>
        <svg width="28" height="28" viewBox="0 0 28 28">
          <circle cx="14" cy="14" r="12" stroke="#FFFFFF" strokeWidth="2" fill="none" opacity="0.9"/>
          <circle cx="14" cy="14" r="4.5" fill="#2EB89A"/>
          {[0,60,120,180,240,300].map(deg => {
            const rad = (deg * Math.PI) / 180
            return <line key={deg}
              x1={14 + 5.5 * Math.cos(rad)} y1={14 + 5.5 * Math.sin(rad)}
              x2={14 + 10 * Math.cos(rad)}  y2={14 + 10 * Math.sin(rad)}
              stroke="#FFFFFF" strokeWidth="1.5" opacity="0.8"
            />
          })}
        </svg>
        <span style={nav.appName}>Wheelchair Admin</span>
        <span style={nav.tagline}>SENSE · MAP · NAVIGATE</span>
      </div>
      <div style={nav.right}>
        <span style={nav.user}>{user}</span>
        <button style={nav.logoutBtn} onClick={onLogout}>Sign out</button>
      </div>
    </div>
  )
}

const nav = {
  bar: {
    background: 'linear-gradient(90deg, #122951 0%, #1B3967 100%)',
    padding: '0 28px',
    height: 56,
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'space-between',
    boxShadow: '0 2px 12px rgba(0,0,0,0.2)',
    position: 'sticky', top: 0, zIndex: 100,
  },
  logo: {
    display: 'flex',
    alignItems: 'center',
    gap: 12,
  },
  appName: {
    color: '#FFFFFF',
    fontWeight: 700,
    fontSize: 16,
    letterSpacing: '-0.2px',
  },
  tagline: {
    color: 'rgba(123,184,212,0.7)',
    fontSize: 10,
    letterSpacing: '1.5px',
    fontWeight: 600,
    marginLeft: 4,
  },
  right: {
    display: 'flex',
    alignItems: 'center',
    gap: 14,
  },
  user: {
    color: 'rgba(255,255,255,0.7)',
    fontSize: 13,
  },
  logoutBtn: {
    padding: '5px 14px',
    background: 'rgba(255,255,255,0.1)',
    color: '#FFFFFF',
    borderRadius: 6,
    fontSize: 13,
    border: '1px solid rgba(255,255,255,0.15)',
    transition: 'background 0.2s',
  },
}

// ── Stat card ─────────────────────────────────────────────────────────────────
function StatCard({ label, value, sub, accent = '#1B3967' }) {
  return (
    <div className="card" style={{ padding: '18px 20px', flex: 1, minWidth: 160 }}>
      <div style={{ fontSize: 11, fontWeight: 700, letterSpacing: '0.8px', textTransform: 'uppercase', color: '#6B8CAE', marginBottom: 6 }}>
        {label}
      </div>
      <div style={{ fontSize: 26, fontWeight: 700, color: accent, lineHeight: 1.1 }}>{value}</div>
      {sub && <div style={{ fontSize: 12, color: '#95BDD8', marginTop: 4 }}>{sub}</div>}
    </div>
  )
}

// ── GPS trail chart ───────────────────────────────────────────────────────────
function GpsTrail({ data }) {
  if (!data.length) return (
    <div style={{ height: 200, display: 'flex', alignItems: 'center', justifyContent: 'center', color: '#95BDD8', fontSize: 13 }}>
      No GPS data available
    </div>
  )
  return (
    <ResponsiveContainer width="100%" height={220}>
      <ScatterChart margin={{ top: 8, right: 8, bottom: 8, left: 8 }}>
        <CartesianGrid strokeDasharray="3 3" stroke="#EBF4FA" />
        <XAxis dataKey="x" name="X" tick={{ fontSize: 11, fill: '#6B8CAE' }} tickLine={false} label={{ value: 'X (m)', position: 'insideBottom', offset: -2, fontSize: 11, fill: '#95BDD8' }} />
        <YAxis dataKey="y" name="Y" tick={{ fontSize: 11, fill: '#6B8CAE' }} tickLine={false} label={{ value: 'Y (m)', angle: -90, position: 'insideLeft', offset: 10, fontSize: 11, fill: '#95BDD8' }} />
        <Tooltip formatter={(v) => v.toFixed(3)} contentStyle={{ fontSize: 12, borderColor: '#CBE0F0', borderRadius: 8 }} />
        <Scatter data={data} line={{ stroke: '#2EB89A', strokeWidth: 2 }} fill="#1B3967" shape="circle" />
      </ScatterChart>
    </ResponsiveContainer>
  )
}

// ── Confidence sparkline ──────────────────────────────────────────────────────
function ConfidenceSparkline({ data }) {
  if (!data.length) return null
  return (
    <ResponsiveContainer width="100%" height={100}>
      <LineChart data={data} margin={{ top: 4, right: 4, bottom: 4, left: 4 }}>
        <XAxis hide />
        <YAxis domain={[0, 1]} hide />
        <Tooltip formatter={(v) => (v * 100).toFixed(0) + '%'} contentStyle={{ fontSize: 12, borderColor: '#CBE0F0', borderRadius: 8 }} />
        <Line type="monotone" dataKey="gps_confidence" stroke="#2EB89A" dot={false} strokeWidth={2} name="GPS conf" />
      </LineChart>
    </ResponsiveContainer>
  )
}

// ── Confidence badge ──────────────────────────────────────────────────────────
function ConfBadge({ v }) {
  if (v == null) return <span className="badge badge-muted">—</span>
  if (v >= 0.7)  return <span className="badge badge-teal">{(v * 100).toFixed(0)}%</span>
  if (v >= 0.4)  return <span className="badge badge-warning">{(v * 100).toFixed(0)}%</span>
  return <span className="badge badge-danger">{(v * 100).toFixed(0)}%</span>
}

// ── Main Dashboard ────────────────────────────────────────────────────────────
export default function Dashboard({ user, onLogout }) {
  const [dbReady,   setDbReady]   = useState(null)   // null=checking, true, false
  const [stats,     setStats]     = useState(null)
  const [frames,    setFrames]    = useState([])
  const [trail,     setTrail]     = useState([])
  const [timeline,  setTimeline]  = useState([])
  const [loading,   setLoading]   = useState(true)
  const navigate = useNavigate()

  // First check if DB is available, then load data
  useEffect(() => {
    fetch('/api/status')
      .then(r => r.json())
      .then(s => {
        setDbReady(s.connected)
        if (s.connected) loadData()
        else setLoading(false)
      })
      .catch(() => { setDbReady(false); setLoading(false) })
  }, [])

  function loadData() {
    setLoading(true)
    Promise.all([
      fetch('/api/stats').then(r => r.json()),
      fetch('/api/frames?limit=100').then(r => r.json()),
      fetch('/api/gps-trail?limit=400').then(r => r.json()),
      fetch('/api/confidence-timeline?limit=80').then(r => r.json()),
    ])
    .then(([s, f, t, tl]) => {
      setStats(s)
      setFrames(Array.isArray(f) ? f : [])
      setTrail(Array.isArray(t) ? t : [])
      setTimeline(Array.isArray(tl) ? tl : [])
      setDbReady(true)
    })
    .catch(console.error)
    .finally(() => setLoading(false))
  }

  const fmt = ts => ts ? new Date(ts * 1000).toLocaleString() : '—'
  const dur = (s, e) => {
    if (!s || !e) return '—'
    const sec = e - s
    if (sec < 60) return `${sec.toFixed(0)}s`
    if (sec < 3600) return `${(sec / 60).toFixed(1)} min`
    return `${(sec / 3600).toFixed(2)} h`
  }

  return (
    <div style={{ minHeight: '100vh', background: 'var(--bg)', display: 'flex', flexDirection: 'column' }}>
      <NavBar user={user} onLogout={onLogout} />

      {/* Show setup panel when DB is not connected */}
      {dbReady === false && (
        <DbSetupPanel onConnected={loadData} />
      )}

      {/* Main content — only when DB is ready */}
      {dbReady === true && (
      <div style={{ maxWidth: 1280, margin: '0 auto', padding: '28px 24px', width: '100%' }}>

        {/* Page title */}
        <div style={{ marginBottom: 24 }}>
          <h1 style={{ fontSize: 22, fontWeight: 700, color: '#1B3967' }}>Sensor Frame Archive</h1>
          <p style={{ color: '#6B8CAE', fontSize: 13, marginTop: 2 }}>
            Historical sensor frames saved from the wheelchair system
          </p>
        </div>


        {/* Stats row */}
        {stats && (
          <div style={{ display: 'flex', gap: 14, marginBottom: 24, flexWrap: 'wrap' }}>
            <StatCard label="Total Frames"    value={stats.total_frames ?? 0} />
            <StatCard label="Session Duration" value={dur(stats.first_ts, stats.last_ts)} sub={fmt(stats.first_ts)} />
            <StatCard label="GPS Coverage"
              value={stats.total_frames ? `${((stats.gps_valid_count / stats.total_frames) * 100).toFixed(0)}%` : '—'}
              sub={`avg conf ${stats.avg_gps_conf != null ? (stats.avg_gps_conf * 100).toFixed(0) + '%' : '—'}`}
              accent="#2EB89A"
            />
            <StatCard label="LiDAR Objects"  value={stats.total_lidar_objects  ?? 0} accent="#3E72B8" />
            <StatCard label="Radar Objects"  value={stats.total_radar_objects  ?? 0} accent="#3E72B8" />
            <StatCard label="Camera Objects" value={stats.total_camera_objects ?? 0} accent="#3E72B8" />
          </div>
        )}

        {/* Two-column layout */}
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 380px', gap: 20, alignItems: 'start' }}>

          {/* Frame table */}
          <div className="card" style={{ padding: 0, overflow: 'hidden' }}>
            <div style={{ padding: '16px 20px', borderBottom: '1px solid var(--border)' }}>
              <span style={{ fontWeight: 700, color: '#1B3967', fontSize: 15 }}>Frames</span>
              <span style={{ color: '#6B8CAE', fontSize: 12, marginLeft: 10 }}>
                {frames.length} rows — click a row to inspect
              </span>
            </div>
            {loading ? (
              <div style={{ padding: 40, textAlign: 'center', color: '#95BDD8' }}>Loading…</div>
            ) : frames.length === 0 ? (
              <div style={{ padding: 40, textAlign: 'center', color: '#95BDD8', fontSize: 13 }}>
                No frames yet. Run the simulation to generate data.
              </div>
            ) : (
              <div style={{ overflowY: 'auto', maxHeight: 520 }}>
                <table>
                  <thead>
                    <tr>
                      <th>Bucket</th>
                      <th>Timestamp</th>
                      <th>GPS</th>
                      <th>GPS Conf</th>
                      <th>IMU</th>
                      <th>Enc</th>
                      <th>LiDAR</th>
                      <th>Radar</th>
                      <th>Cam</th>
                    </tr>
                  </thead>
                  <tbody>
                    {frames.map(f => (
                      <tr key={f.frame_bucket}
                        onClick={() => navigate(`/frame/${f.frame_bucket}`)}
                        style={{ cursor: 'pointer' }}
                      >
                        <td style={{ fontFamily: 'monospace', color: '#3E72B8', fontSize: 12 }}>{f.frame_bucket}</td>
                        <td style={{ whiteSpace: 'nowrap', fontSize: 12 }}>{fmt(f.timestamp)}</td>
                        <td>{f.gps_valid ? <span className="badge badge-teal">✓</span> : <span className="badge badge-muted">—</span>}</td>
                        <td><ConfBadge v={f.gps_confidence} /></td>
                        <td style={{ color: f.imu_count > 0 ? '#1A2E4A' : '#95BDD8' }}>{f.imu_count || '—'}</td>
                        <td style={{ color: f.enc_count > 0 ? '#1A2E4A' : '#95BDD8' }}>{f.enc_count || '—'}</td>
                        <td style={{ color: f.lidar_count > 0 ? '#1A2E4A' : '#95BDD8' }}>{f.lidar_count || '—'}</td>
                        <td style={{ color: f.radar_count > 0 ? '#1A2E4A' : '#95BDD8' }}>{f.radar_count || '—'}</td>
                        <td style={{ color: f.camera_count > 0 ? '#1A2E4A' : '#95BDD8' }}>{f.camera_count || '—'}</td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            )}
          </div>

          {/* Right column: GPS trail + confidence chart */}
          <div style={{ display: 'flex', flexDirection: 'column', gap: 16 }}>
            <div className="card" style={{ padding: '16px 18px' }}>
              <div style={{ fontWeight: 700, color: '#1B3967', fontSize: 14, marginBottom: 12 }}>
                GPS Path
              </div>
              <GpsTrail data={trail} />
            </div>

            <div className="card" style={{ padding: '16px 18px' }}>
              <div style={{ fontWeight: 700, color: '#1B3967', fontSize: 14, marginBottom: 8 }}>
                GPS Confidence Timeline
              </div>
              <ConfidenceSparkline data={timeline} />
            </div>
          </div>

        </div>
      </div>
      )}
    </div>
  )
}
