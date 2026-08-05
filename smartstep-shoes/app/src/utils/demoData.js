import { addDocument, setDocument } from '../firestoreRest.js'

const EXERCISES = ['lunge', 'mountainClimber', 'situp']
const DAY_MS = 24 * 60 * 60 * 1000

const DEMO_COACH_STATUSES = [
  { state: 'training', feedbackCategory: 'stable', message: 'מצוין! תנועה יציבה ומאוזנת' },
  { state: 'training', feedbackCategory: 'too_easy', message: 'אפשר להעלות קצת את הקצב' },
  { state: 'training', feedbackCategory: 'too_hard', message: 'האטי מעט את הקצב' },
  { state: 'alert', feedbackCategory: 'unstable', message: 'שימי לב ליציבות הקרסול' },
  { state: 'alert', feedbackCategory: 'discomfort', message: 'מומלץ לעצור ולנוח רגע' },
  { state: 'idle', feedbackCategory: 'uncertain', message: 'ממתין לתחילת אימון' },
]

function randomInt(min, max) {
  return Math.floor(Math.random() * (max - min + 1)) + min
}

function buildFakeSession(daysAgoMax) {
  const exercise = EXERCISES[randomInt(0, EXERCISES.length - 1)]
  const reps = randomInt(6, 12)
  const scores = Array.from({ length: reps }, () => randomInt(55, 100))
  const avgScore = Math.round(scores.reduce((sum, s) => sum + s, 0) / scores.length)
  const timestamp = Date.now() - randomInt(0, daysAgoMax * DAY_MS)

  return { exercise, timestamp, reps, scores, avgScore }
}

export async function seedDemoSessions() {
  const count = randomInt(5, 10)
  const writes = Array.from({ length: count }, () => {
    const session = buildFakeSession(14)
    return addDocument('sessions', session)
  })

  const coachStatus = DEMO_COACH_STATUSES[randomInt(0, DEMO_COACH_STATUSES.length - 1)]
  writes.push(setDocument('state/coachStatus', { ...coachStatus, timestamp: Date.now() }))

  await Promise.all(writes)
}
