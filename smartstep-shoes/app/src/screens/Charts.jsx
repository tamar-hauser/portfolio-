import { useMemo, useState } from 'react'
import {
  CartesianGrid,
  Line,
  LineChart,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis,
} from 'recharts'
import { TrendingUp } from 'lucide-react'
import { useSessions } from '../hooks/useSessions.js'
import EmptyState from '../components/EmptyState.jsx'
import { EXERCISE_LABELS, exerciseLabel, formatDate } from '../utils/format.js'

const FILTERS = [{ value: 'all', label: 'הכל' }, ...Object.entries(EXERCISE_LABELS).map(([value, label]) => ({ value, label }))]

export default function Charts() {
  const { sessions, loading } = useSessions()
  const [filter, setFilter] = useState('all')

  const chartData = useMemo(() => {
    const filtered = filter === 'all' ? sessions : sessions.filter((s) => s.exercise === filter)
    return [...filtered]
      .sort((a, b) => a.timestamp - b.timestamp)
      .map((s) => ({
        timestamp: s.timestamp,
        date: formatDate(s.timestamp),
        avgScore: s.avgScore,
        exercise: exerciseLabel(s.exercise),
      }))
  }, [sessions, filter])

  if (loading) {
    return <div className="screen"><p className="muted">טוען...</p></div>
  }

  return (
    <div className="screen">
      <h2 className="screen-title">התקדמות לאורך זמן</h2>

      <div className="filter-row">
        {FILTERS.map((f) => (
          <button
            key={f.value}
            type="button"
            className={'filter-chip' + (filter === f.value ? ' active' : '')}
            onClick={() => setFilter(f.value)}
          >
            {f.label}
          </button>
        ))}
      </div>

      {chartData.length === 0 ? (
        <EmptyState icon={<TrendingUp />} title="אין נתונים מספיקים להצגת גרף" />
      ) : (
        <div className="chart-wrapper">
          <ResponsiveContainer width="100%" height={320}>
            <LineChart data={chartData} margin={{ top: 10, right: 10, bottom: 10, left: -10 }}>
              <CartesianGrid strokeDasharray="3 3" stroke="var(--color-border)" />
              <XAxis dataKey="date" tick={{ fontSize: 12 }} />
              <YAxis domain={[0, 100]} tick={{ fontSize: 12 }} />
              <Tooltip
                labelFormatter={(label, payload) => payload?.[0]?.payload ? `${label} - ${payload[0].payload.exercise}` : label}
                formatter={(value) => [value, 'ציון ממוצע']}
              />
              <Line
                type="monotone"
                dataKey="avgScore"
                stroke="var(--color-primary)"
                strokeWidth={2}
                dot={{ r: 3 }}
                activeDot={{ r: 5 }}
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
      )}
    </div>
  )
}
