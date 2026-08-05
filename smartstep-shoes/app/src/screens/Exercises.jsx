import { useState } from 'react'
import { EXERCISE_LABELS } from '../utils/format.js'
import ExerciseVideoCard from '../components/ExerciseVideoCard.jsx'
import { setDocument } from '../firestoreRest.js'

const EXERCISE_HINTS = {
  lunge: 'הנעליים בודקות א-סימטריה בין הרגל הקדמית לאחורית, ויציבות הקרסול (IMU) בנקודת השפל של התרגיל.',
  mountainClimber: 'הנעליים בודקות את קצב החלפת המשקל בין רגל לרגל וסימטריה בין הצדדים תוך כדי התנועה המהירה.',
  situp: 'הנעליים בודקות שהלחץ נשאר יציב ומחולק שווה בין שתי הרגליים לאורך כל החזרה, כי כפות הרגליים לא אמורות לזוז בתרגיל הזה.',
}

const EXERCISES = Object.keys(EXERCISE_LABELS)

export default function Exercises() {
  const [startingExercise, setStartingExercise] = useState(null)
  const [statusByExercise, setStatusByExercise] = useState({})

  async function handleStartLesson(exercise) {
    setStartingExercise(exercise)
    setStatusByExercise((prev) => ({ ...prev, [exercise]: null }))
    try {
      await setDocument('state/currentLesson', { exercise, requestedAt: Date.now() })
      setStatusByExercise((prev) => ({ ...prev, [exercise]: 'נשלח! ודאי שהנעליים דלוקות ומחוברות ל-C6.' }))
    } catch (err) {
      setStatusByExercise((prev) => ({ ...prev, [exercise]: 'שליחה נכשלה - בדקי חיבור לאינטרנט ונסי שוב.' }))
    } finally {
      setStartingExercise(null)
    }
  }

  return (
    <div className="screen">
      <h2 className="screen-title">תרגילים</h2>
      <p className="settings-note">
        סרטון הדגמה של המאמנת לביצוע נכון של כל תרגיל, לפני שמתחילים אימון עם הנעליים
      </p>

      <div className="exercise-list">
        {EXERCISES.map((exercise) => (
          <ExerciseVideoCard
            key={exercise}
            exercise={exercise}
            label={EXERCISE_LABELS[exercise]}
            hint={EXERCISE_HINTS[exercise]}
            onStartLesson={handleStartLesson}
            isStarting={startingExercise === exercise}
            startStatus={statusByExercise[exercise]}
          />
        ))}
      </div>
    </div>
  )
}
