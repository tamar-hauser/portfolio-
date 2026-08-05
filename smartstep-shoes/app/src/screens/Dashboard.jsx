import { Activity } from 'lucide-react'
import { useSessions } from '../hooks/useSessions.js'
import { useLiveData } from '../hooks/useLiveData.js'
import EmptyState from '../components/EmptyState.jsx'
import { exerciseLabel, formatDateTime, scoreColor } from '../utils/format.js'
import DemoDataButton from '../components/DemoDataButton.jsx'
import CoachStatusCard from '../components/CoachStatusCard.jsx'
import FootPressureMap from '../components/FootPressureMap.jsx'
import LiveSimToggle from '../components/LiveSimToggle.jsx'

export default function Dashboard() {
  const { sessions, loading } = useSessions()
  const { liveData } = useLiveData()

  if (loading) {
    return <div className="screen"><p className="muted">טוען...</p></div>
  }

  const latest = sessions[0]

  return (
    <div className="screen">
      <CoachStatusCard />

      <FootPressureMap liveData={liveData} />

      <LiveSimToggle />

      <h2 className="screen-title">אימון אחרון</h2>

      {!latest ? (
        <EmptyState
          icon={<Activity />}
          title="עדיין לא בוצע אימון"
          subtitle="ברגע שהנעליים יתחילו לשלוח נתונים, האימון האחרון יופיע כאן"
        />
      ) : (
        <div className="latest-card">
          <div className="latest-card-header">
            <span className="latest-exercise">{exerciseLabel(latest.exercise)}</span>
            <span className="latest-date">{formatDateTime(latest.timestamp)}</span>
          </div>

          <div className="latest-score" style={{ color: scoreColor(latest.avgScore) }}>
            {latest.avgScore}
            <span className="latest-score-unit">/100</span>
          </div>

          <div className="latest-stats">
            <div className="stat-box">
              <span className="stat-value">{latest.reps}</span>
              <span className="stat-label">חזרות</span>
            </div>
            <div className="stat-box">
              <span className="stat-value">{latest.scores?.length ?? 0}</span>
              <span className="stat-label">ציונים נרשמו</span>
            </div>
          </div>
        </div>
      )}

      <DemoDataButton />
    </div>
  )
}
