import { useState } from 'react'
import { ChevronDown, ChevronUp, ClipboardList } from 'lucide-react'
import { useSessions } from '../hooks/useSessions.js'
import EmptyState from '../components/EmptyState.jsx'
import { exerciseLabel, formatDateTime, scoreColor } from '../utils/format.js'

export default function History() {
  const { sessions, loading } = useSessions()
  const [expandedId, setExpandedId] = useState(null)

  if (loading) {
    return <div className="screen"><p className="muted">טוען...</p></div>
  }

  return (
    <div className="screen">
      <h2 className="screen-title">היסטוריית אימונים</h2>

      {sessions.length === 0 ? (
        <EmptyState icon={<ClipboardList />} title="אין עדיין אימונים בהיסטוריה" />
      ) : (
        <ul className="session-list">
          {sessions.map((session) => {
            const isExpanded = expandedId === session.id
            return (
              <li key={session.id} className="session-item">
                <button
                  type="button"
                  className="session-row"
                  onClick={() => setExpandedId(isExpanded ? null : session.id)}
                >
                  <span className="session-exercise">{exerciseLabel(session.exercise)}</span>
                  <span className="session-date">{formatDateTime(session.timestamp)}</span>
                  <span className="session-reps">{session.reps} חזרות</span>
                  <span className="session-score" style={{ color: scoreColor(session.avgScore) }}>
                    {session.avgScore}
                  </span>
                  <span className="session-caret">
                    {isExpanded ? <ChevronUp size={16} /> : <ChevronDown size={16} />}
                  </span>
                </button>

                {isExpanded && (
                  <div className="session-details">
                    <span className="session-details-label">ציונים לפי חזרה:</span>
                    <div className="rep-scores">
                      {(session.scores || []).map((score, index) => (
                        <span key={index} className="rep-score" style={{ color: scoreColor(score) }}>
                          {score}
                        </span>
                      ))}
                    </div>
                  </div>
                )}
              </li>
            )
          })}
        </ul>
      )}
    </div>
  )
}
