import { useCoachStatus } from '../hooks/useCoachStatus.js'
import { feedbackColor, feedbackLabel, shoeStateLabel } from '../utils/coach.js'

const STALE_STATUS_MS = 15000

export default function CoachStatusCard() {
  const { status, loading } = useCoachStatus()

  if (loading || !status) return null

  const isStale = !status.timestamp || Date.now() - status.timestamp > STALE_STATUS_MS
  const shoesDisconnected = isStale || status.shoesConnected === false

  if (shoesDisconnected) {
    return (
      <div className="coach-card coach-card--disconnected">
        <div className="coach-card-header">
          <span className="coach-card-title">מאמן וירטואלי</span>
          <span className="coach-state-badge">אין חיבור</span>
        </div>
        <p className="coach-message">
          {isStale ? 'אין תקשורת עם ה-C6 - ודאי שהוא מחובר ל-WiFi' : 'אין חיבור לנעליים - ודאי שהן דלוקות'}
        </p>
      </div>
    )
  }

  const color = feedbackColor(status.feedbackCategory)
  const cardClass = 'coach-card' + (status.state === 'alert' ? ' coach-card--alert' : '')

  return (
    <div className={cardClass} style={{ borderColor: color }}>
      <div className="coach-card-header">
        <span className="coach-card-title">מאמן וירטואלי</span>
        <span className="coach-state-badge">{shoeStateLabel(status.state)}</span>
      </div>
      <div className="coach-card-body">
        <span className="coach-category-dot" style={{ background: color }} />
        <span className="coach-category-label" style={{ color }}>
          {feedbackLabel(status.feedbackCategory)}
        </span>
      </div>
      {status.message && <p className="coach-message">{status.message}</p>}
    </div>
  )
}
