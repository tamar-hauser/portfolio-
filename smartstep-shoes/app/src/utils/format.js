export const EXERCISE_LABELS = {
  lunge: 'לאנג\'',
  mountainClimber: 'מטפסי הרים',
  situp: 'סיט אפ',
}

export function exerciseLabel(exercise) {
  return EXERCISE_LABELS[exercise] || exercise
}

export function formatDate(timestamp) {
  return new Date(timestamp).toLocaleDateString('he-IL', {
    day: '2-digit',
    month: '2-digit',
    year: 'numeric',
  })
}

export function formatDateTime(timestamp) {
  return new Date(timestamp).toLocaleString('he-IL', {
    day: '2-digit',
    month: '2-digit',
    year: 'numeric',
    hour: '2-digit',
    minute: '2-digit',
  })
}

export function scoreColor(score) {
  if (score >= 80) return 'var(--color-good)'
  if (score >= 60) return 'var(--color-warn)'
  return 'var(--color-bad)'
}
