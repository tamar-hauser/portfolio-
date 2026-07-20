import { useState, useEffect } from 'react'
import { useParams, useNavigate } from 'react-router-dom'
import ConfidenceBar from '../components/ConfidenceBar.jsx'

// ── Helpers ──────────────────────────────────────────────────────────────────
const fmt3  = v => (v != null ? Number(v).toFixed(3) : '—')
const fmt2  = v => (v != null ? Number(v).toFixed(2) : '—')
const fmtTs = ts => ts ? new Date(ts * 1000).toLocaleString() : '—'

function Row({ label, value, unit = '', mono = false }) {
  return (
    <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', padding: '7px 0', borderBottom: '1px solid #F0F6FB' }}>
      <span style={{ color: '#6B8CAE', fontSize: 12, fontWeight: 500 }}>{label}</span>
      <span style={{ fontFamily: mono ? 'monospace' : 'inherit', fontSize: 13, fontWeight: 600, color: '#1A2E4A' }}>
        {value}{unit && <span style={{ color: '#95BDD8', fontWeight: 400, marginLeft: 3 }}>{unit}</span>}
      </span>
    </div>
  )
}

function SectionCard({ title, accent = '#1B3967', icon, children, badge }) {
  return (
    <div className="card" style={{ padding: '18px 22px' }}>
      <div style={{ display: 'flex', alignItems: 'center', gap: 10, marginBottom: 16, borderBottom: '2px solid', borderImage: `linear-gradient(90deg, ${accent}, transparent) 1` }}>
        <span style={{ fontSize: 18 }}>{icon}</span>
        <span style={{ fontWeight: 700, fontSize: 15, color: accent }}>{title}</span>
        {badge != null && (
          <span style={{ marginLeft: 'auto', fontSize: 11, fontWeight: 700, padding: '2px 10px', borderRadius: 12, background: '#EFF6FC', color: '#6B8CAE' }}>
            {badge}
          </span>
        )}
      </div>
      {children}
    </div>
  )
}

// ── GPS Card ─────────────────────────────────────────────────────────────────
function GpsCard({ frame }) {
  const valid = !!frame.gps_valid
  return (
    <SectionCard title="GPS" icon="🛰" accent="#2EB89A"
      badge={valid ? 'VALID' : 'NO FIX'}
    >
      <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '0 32px' }}>
        <div>
          <Row label="Latitude"   value={fmt3(frame.gps_latitude)}  unit="°" mono />
          <Row label="Longitude"  value={fmt3(frame.gps_longitude)} unit="°" mono />
          <Row label="Altitude"   value={fmt3(frame.gps_altitude)}  unit="m" />
        </div>
        <div>
          <Row label="Speed"     value={fmt2(frame.gps_speed)}   unit="m/s" />
          <Row label="Heading"   value={fmt2(frame.gps_heading)} unit="°" />
          <Row label="Valid"     value={valid ? '✓ Active' : '✗ Void'} />
        </div>
      </div>
      <div style={{ marginTop: 14 }}>
        <div style={{ fontSize: 11, fontWeight: 700, letterSpacing: '0.6px', textTransform: 'uppercase', color: '#6B8CAE', marginBottom: 6 }}>
          Confidence
        </div>
        <ConfidenceBar value={frame.gps_confidence} />
      </div>
    </SectionCard>
  )
}

// ── IMU Card ─────────────────────────────────────────────────────────────────
function ImuCard({ frame }) {
  return (
    <SectionCard title="IMU" icon="🔄" accent="#3E72B8" badge={`${frame.imu_count ?? 0} samples`}>
      <Row label="Avg Pitch"    value={fmt3(frame.imu_avg_pitch)} unit="rad" />
      <Row label="Avg Yaw"      value={fmt3(frame.imu_avg_yaw)}   unit="rad" />
      <Row label="Avg ω (Vyaw)" value={fmt3(frame.imu_avg_vyaw)} unit="rad/s" />
      <Row label="Avg Ax"       value={fmt3(frame.imu_avg_ax)}   unit="m/s²" />
    </SectionCard>
  )
}

// ── Encoder Card ─────────────────────────────────────────────────────────────
function EncoderCard({ frame }) {
  return (
    <SectionCard title="Encoders" icon="⚙️" accent="#7BB8D4" badge={`${frame.enc_count ?? 0} samples`}>
      <Row label="Avg Linear Velocity"  value={fmt3(frame.enc_avg_v_lin)} unit="m/s" />
      <Row label="Avg Angular Velocity" value={fmt3(frame.enc_avg_v_ang)} unit="rad/s" />
    </SectionCard>
  )
}

// ── Object table (LiDAR / Radar / Camera) ────────────────────────────────────
function ObjectTable({ columns, rows }) {
  if (!rows || rows.length === 0) {
    return <div style={{ padding: '16px 0', textAlign: 'center', color: '#95BDD8', fontSize: 13 }}>No objects detected</div>
  }
  return (
    <div style={{ overflowX: 'auto' }}>
      <table>
        <thead>
          <tr>{columns.map(c => <th key={c.key}>{c.label}</th>)}</tr>
        </thead>
        <tbody>
          {rows.map((row, i) => (
            <tr key={i}>
              {columns.map(c => (
                <td key={c.key}>
                  {c.key === 'confidence'
                    ? <ConfidenceBar value={row[c.key]} />
                    : c.fmt ? c.fmt(row[c.key]) : (row[c.key] ?? '—')}
                </td>
              ))}
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  )
}

function LidarCard({ objects }) {
  const cols = [
    { key: 'obj_index', label: '#' },
    { key: 'pos_x',   label: 'X (m)',  fmt: fmt2 },
    { key: 'pos_y',   label: 'Y (m)',  fmt: fmt2 },
    { key: 'pos_z',   label: 'Z (m)',  fmt: fmt2 },
    { key: 'yaw',     label: 'Yaw',    fmt: fmt2 },
    { key: 'length',  label: 'L (m)',  fmt: fmt2 },
    { key: 'width',   label: 'W (m)',  fmt: fmt2 },
    { key: 'height',  label: 'H (m)',  fmt: fmt2 },
    { key: 'confidence', label: 'Confidence' },
  ]
  return (
    <SectionCard title="LiDAR Objects" icon="📡" accent="#3E72B8" badge={objects?.length ?? 0}>
      <ObjectTable columns={cols} rows={objects} />
    </SectionCard>
  )
}

function RadarCard({ objects }) {
  const cols = [
    { key: 'obj_index', label: '#' },
    { key: 'range_m',  label: 'Range (m)', fmt: fmt2 },
    { key: 'pos_x',    label: 'X',         fmt: fmt2 },
    { key: 'pos_y',    label: 'Y',         fmt: fmt2 },
    { key: 'vel_x',    label: 'Vx (m/s)',  fmt: fmt2 },
    { key: 'vel_y',    label: 'Vy (m/s)',  fmt: fmt2 },
    { key: 'rcs',      label: 'RCS',       fmt: fmt2 },
    { key: 'confidence', label: 'Confidence' },
  ]
  return (
    <SectionCard title="Radar Objects" icon="〰️" accent="#1B3967" badge={objects?.length ?? 0}>
      <ObjectTable columns={cols} rows={objects} />
    </SectionCard>
  )
}

function CameraCard({ objects }) {
  const cols = [
    { key: 'obj_index',  label: '#' },
    { key: 'type_label', label: 'Label' },
    { key: 'pos_x',      label: 'X (m)', fmt: fmt2 },
    { key: 'pos_y',      label: 'Y (m)', fmt: fmt2 },
    { key: 'pos_z',      label: 'Z (m)', fmt: fmt2 },
    { key: 'length',     label: 'L',     fmt: fmt2 },
    { key: 'width',      label: 'W',     fmt: fmt2 },
    { key: 'height',     label: 'H',     fmt: fmt2 },
    { key: 'confidence', label: 'Confidence' },
  ]
  return (
    <SectionCard title="Camera Objects" icon="📷" accent="#7BB8D4" badge={objects?.length ?? 0}>
      <ObjectTable columns={cols} rows={objects} />
    </SectionCard>
  )
}

// ── Main ─────────────────────────────────────────────────────────────────────
export default function FrameDetail() {
  const { bucket } = useParams()
  const navigate   = useNavigate()
  const [data,    setData]    = useState(null)
  const [loading, setLoading] = useState(true)
  const [error,   setError]   = useState('')

  useEffect(() => {
    fetch(`/api/frames/${bucket}`)
      .then(r => r.ok ? r.json() : r.json().then(e => Promise.reject(e.error)))
      .then(setData)
      .catch(e => setError(String(e)))
      .finally(() => setLoading(false))
  }, [bucket])

  if (loading) return (
    <div style={{ minHeight: '100vh', display: 'flex', alignItems: 'center', justifyContent: 'center', color: '#6B8CAE' }}>
      Loading frame {bucket}…
    </div>
  )

  if (error) return (
    <div style={{ minHeight: '100vh', display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', gap: 16 }}>
      <div style={{ color: '#D94F4F', fontSize: 15 }}>⚠ {error}</div>
      <button onClick={() => navigate('/')} style={backBtnStyle}>← Back to Dashboard</button>
    </div>
  )

  return (
    <div style={{ minHeight: '100vh', background: 'var(--bg)' }}>
      {/* Top bar */}
      <div style={headerBar}>
        <button onClick={() => navigate('/')} style={backBtnStyle}>← Dashboard</button>
        <div style={{ display: 'flex', alignItems: 'center', gap: 16 }}>
          <div>
            <span style={{ fontSize: 12, color: '#95BDD8' }}>Frame bucket </span>
            <span style={{ fontFamily: 'monospace', fontWeight: 700, color: '#FFFFFF', fontSize: 16 }}>{data.frame_bucket}</span>
          </div>
          <div style={{ color: 'rgba(255,255,255,0.6)', fontSize: 13 }}>{fmtTs(data.timestamp)}</div>
          <span className={`badge ${data.is_processed ? 'badge-teal' : 'badge-warning'}`}>
            {data.is_processed ? '✓ Processed' : '⏳ Pending'}
          </span>
        </div>
      </div>

      {/* Content */}
      <div style={{ maxWidth: 1100, margin: '0 auto', padding: '28px 24px', display: 'flex', flexDirection: 'column', gap: 18 }}>

        {/* GPS — full width */}
        <GpsCard frame={data} />

        {/* IMU + Encoder — side by side */}
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 18 }}>
          <ImuCard     frame={data} />
          <EncoderCard frame={data} />
        </div>

        {/* Object tables */}
        <LidarCard  objects={data.lidar}  />
        <RadarCard  objects={data.radar}  />
        <CameraCard objects={data.camera} />
      </div>
    </div>
  )
}

const headerBar = {
  background: 'linear-gradient(90deg, #122951 0%, #1B3967 100%)',
  padding: '14px 28px',
  display: 'flex',
  alignItems: 'center',
  justifyContent: 'space-between',
  boxShadow: '0 2px 12px rgba(0,0,0,0.2)',
  position: 'sticky',
  top: 0,
  zIndex: 100,
}

const backBtnStyle = {
  padding: '7px 16px',
  background: 'rgba(255,255,255,0.12)',
  color: '#FFFFFF',
  border: '1px solid rgba(255,255,255,0.18)',
  borderRadius: 7,
  fontSize: 13,
  fontWeight: 600,
  cursor: 'pointer',
}
