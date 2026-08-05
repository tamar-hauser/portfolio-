import { useState } from 'react'
import { Video, Play } from 'lucide-react'

export default function ExerciseVideoCard({ exercise, label, hint, onStartLesson, isStarting, startStatus }) {
  const [videoMissing, setVideoMissing] = useState(false)

  return (
    <div className="exercise-card">
      <div className="exercise-video-wrapper">
        {videoMissing ? (
          <div className="exercise-video-placeholder">
            <span className="exercise-video-placeholder-icon">
              <Video size={28} />
            </span>
            <span>אין עדיין סרטון הדגמה</span>
          </div>
        ) : (
          <video className="exercise-video" controls preload="metadata" onError={() => setVideoMissing(true)}>
            <source src={`/videos/${exercise}.mp4`} type="video/mp4" />
          </video>
        )}
      </div>
      <div className="exercise-info">
        <span className="exercise-name">{label}</span>
        <p className="exercise-hint">{hint}</p>
        <button
          type="button"
          className="start-lesson-button"
          onClick={() => onStartLesson(exercise)}
          disabled={isStarting}
        >
          <Play size={16} />
          {isStarting ? 'שולחת ל-C6...' : 'התחילי שיעור'}
        </button>
        {startStatus && <p className="start-lesson-status">{startStatus}</p>}
      </div>
    </div>
  )
}
