import { useEffect, useState } from 'react'
import { getDocument, setDocument } from '../firestoreRest.js'
import { EXERCISE_LABELS } from '../utils/format.js'
import TrainerUploadRow from '../components/TrainerUploadRow.jsx'

const EXERCISES = Object.keys(EXERCISE_LABELS)

function readFileAsText(file) {
  return new Promise((resolve, reject) => {
    const reader = new FileReader()
    reader.onload = () => resolve(reader.result)
    reader.onerror = reject
    reader.readAsText(file)
  })
}

export default function Trainer() {
  const [referenceStatus, setReferenceStatus] = useState({})
  const [loading, setLoading] = useState(true)

  async function refresh() {
    const entries = await Promise.all(
      EXERCISES.map(async (exercise) => {
        const doc = await getDocument(`referenceFrames/${exercise}`)
        return [exercise, Boolean(doc)]
      }),
    )
    setReferenceStatus(Object.fromEntries(entries))
    setLoading(false)
  }

  useEffect(() => {
    refresh()
  }, [])

  async function handleReferenceUpload(exercise, file) {
    const content = await readFileAsText(file)
    const format = file.name.endsWith('.csv') ? 'csv' : 'json'
    await setDocument(`referenceFrames/${exercise}`, { content, format, uploadedAt: Date.now() })
    await refresh()
  }

  if (loading) {
    return <div className="screen"><p className="muted">טוען...</p></div>
  }

  return (
    <div className="screen">
      <h2 className="screen-title">אזור מאמן</h2>
      <p className="settings-note">
        העלאת קובץ ייחוס חיישנים (Reference - דגימות FSR+IMU) לכל תרגיל, שה-ESP32-C6 ישווה אליו ביצוע חי.
        סרטוני הדגמה מתווספים ידנית לתיקיית public/videos באפליקציה.
      </p>

      <div className="trainer-list">
        {EXERCISES.map((exercise) => (
          <div key={exercise} className="trainer-card">
            <span className="trainer-exercise-name">{EXERCISE_LABELS[exercise]}</span>
            <TrainerUploadRow
              label="קובץ ייחוס חיישנים (JSON/CSV)"
              accept=".json,.csv"
              hasFile={Boolean(referenceStatus[exercise])}
              onUpload={(file) => handleReferenceUpload(exercise, file)}
            />
          </div>
        ))}
      </div>
    </div>
  )
}
