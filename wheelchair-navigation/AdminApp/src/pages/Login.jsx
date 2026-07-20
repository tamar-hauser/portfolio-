import { useState } from 'react'

// Decorative wheel SVG matching the logo style
function WheelIcon({ size = 56 }) {
  const r = size / 2
  const spokes = [0, 60, 120, 180, 240, 300]
  return (
    <svg width={size} height={size} viewBox={`0 0 ${size} ${size}`}>
      <circle cx={r} cy={r} r={r - 3} stroke="#FFFFFF" strokeWidth="2.5" fill="none" opacity="0.9" />
      <circle cx={r} cy={r} r={r * 0.32} fill="#2EB89A" />
      {spokes.map(deg => {
        const rad = (deg * Math.PI) / 180
        const inner = r * 0.38
        const outer = r * 0.72
        return (
          <line key={deg}
            x1={r + inner * Math.cos(rad)} y1={r + inner * Math.sin(rad)}
            x2={r + outer * Math.cos(rad)} y2={r + outer * Math.sin(rad)}
            stroke="#FFFFFF" strokeWidth="2" opacity="0.85"
          />
        )
      })}
    </svg>
  )
}

export default function Login({ onLogin }) {
  const [username, setUsername] = useState('')
  const [password, setPassword] = useState('')
  const [error, setError]       = useState('')
  const [loading, setLoading]   = useState(false)

  async function handleSubmit(e) {
    e.preventDefault()
    if (!username || !password) return
    setLoading(true)
    setError('')
    try {
      const res  = await fetch('/api/auth/login', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ username, password }),
      })
      const data = await res.json()
      if (data.ok) onLogin(data.username)
      else setError('Incorrect username or password')
    } catch {
      setError('Cannot reach the server. Make sure npm run dev is running.')
    } finally {
      setLoading(false)
    }
  }

  return (
    <div style={css.page}>
      {/* Background decorative circles */}
      <div style={{ ...css.circle, width: 500, height: 500, top: -180, left: -180, opacity: 0.06 }} />
      <div style={{ ...css.circle, width: 700, height: 700, bottom: -260, right: -260, opacity: 0.04 }} />
      <div style={{ ...css.circle, width: 180, height: 180, top: '18%', right: '12%', borderStyle: 'dashed', opacity: 0.12 }} />
      <div style={{ ...css.circle, width: 80, height: 80, top: '55%', left: '8%', opacity: 0.09 }} />

      <div style={css.card}>
        {/* Logo area */}
        <div style={css.logoArea}>
          <div style={css.iconRing}>
            <WheelIcon size={52} />
          </div>
          <div style={css.appName}>Autonomous Wheelchair</div>
          <div style={css.divider} />
          <div style={css.tagline}>SENSE · MAP · NAVIGATE</div>
          <div style={css.subLabel}>Sensor Management Console</div>
        </div>

        {/* Form */}
        <form style={css.form} onSubmit={handleSubmit}>
          <div>
            <label style={css.label}>Username</label>
            <input
              style={css.input}
              type="text"
              value={username}
              onChange={e => setUsername(e.target.value)}
              placeholder="admin"
              autoFocus
              autoComplete="username"
            />
          </div>
          <div>
            <label style={css.label}>Password</label>
            <input
              style={css.input}
              type="password"
              value={password}
              onChange={e => setPassword(e.target.value)}
              placeholder="••••••••"
              autoComplete="current-password"
            />
          </div>

          {error && (
            <div style={css.errorBox}>
              <span style={{ fontSize: 15, marginRight: 6 }}>⚠</span>
              {error}
            </div>
          )}

          <button style={{ ...css.btn, opacity: loading ? 0.7 : 1 }} type="submit" disabled={loading}>
            {loading ? 'Signing in…' : 'Sign In'}
          </button>
        </form>

        <div style={css.hint}>Default: admin / wheelchair2024</div>
      </div>
    </div>
  )
}

const css = {
  page: {
    minHeight: '100vh',
    background: 'linear-gradient(145deg, #0E2147 0%, #1B3967 45%, #1E4580 100%)',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    position: 'relative',
    overflow: 'hidden',
  },
  circle: {
    position: 'absolute',
    border: '1.5px solid #7BB8D4',
    borderRadius: '50%',
    pointerEvents: 'none',
  },
  card: {
    background: '#FFFFFF',
    borderRadius: 18,
    padding: '44px 40px 32px',
    width: '100%',
    maxWidth: 400,
    boxShadow: '0 24px 64px rgba(0,0,0,0.35)',
    position: 'relative',
    zIndex: 1,
  },
  logoArea: {
    textAlign: 'center',
    marginBottom: 32,
  },
  iconRing: {
    display: 'inline-flex',
    alignItems: 'center',
    justifyContent: 'center',
    width: 72,
    height: 72,
    background: 'linear-gradient(135deg, #1B3967, #3E72B8)',
    borderRadius: '50%',
    marginBottom: 14,
    boxShadow: '0 4px 16px rgba(27,57,103,0.25)',
  },
  appName: {
    fontSize: 20,
    fontWeight: 700,
    color: '#1B3967',
    letterSpacing: '-0.2px',
  },
  divider: {
    width: 44,
    height: 2.5,
    background: 'linear-gradient(90deg, #1B3967, #2EB89A)',
    borderRadius: 2,
    margin: '10px auto',
  },
  tagline: {
    fontSize: 10,
    fontWeight: 700,
    letterSpacing: '2.5px',
    color: '#6B8CAE',
  },
  subLabel: {
    fontSize: 12,
    color: '#95BDD8',
    marginTop: 6,
  },
  form: {
    display: 'flex',
    flexDirection: 'column',
    gap: 16,
  },
  label: {
    display: 'block',
    fontSize: 11,
    fontWeight: 700,
    color: '#1A2E4A',
    letterSpacing: '0.7px',
    textTransform: 'uppercase',
    marginBottom: 6,
  },
  input: {
    width: '100%',
    padding: '10px 14px',
    border: '1.5px solid #CBE0F0',
    borderRadius: 8,
    fontSize: 14,
    color: '#1A2E4A',
    outline: 'none',
    background: '#F8FBFE',
    transition: 'border-color 0.2s',
  },
  errorBox: {
    display: 'flex',
    alignItems: 'center',
    background: '#FEF2F2',
    border: '1px solid #FCA5A5',
    borderRadius: 8,
    padding: '10px 14px',
    color: '#D94F4F',
    fontSize: 13,
  },
  btn: {
    width: '100%',
    padding: '12px',
    background: 'linear-gradient(135deg, #1B3967 0%, #3E72B8 100%)',
    color: '#FFFFFF',
    borderRadius: 8,
    fontSize: 14,
    fontWeight: 600,
    letterSpacing: '0.3px',
    marginTop: 4,
    transition: 'opacity 0.2s, transform 0.1s',
    boxShadow: '0 4px 14px rgba(27,57,103,0.25)',
  },
  hint: {
    textAlign: 'center',
    marginTop: 20,
    fontSize: 11,
    color: '#95BDD8',
  },
}
