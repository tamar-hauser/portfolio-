
export const SHOE_STATE_LABELS = {
  idle: 'ממתין',
  calibrating: 'כיול',
  training: 'באימון',
  alert: 'התראה',
}

export const FEEDBACK_LABELS = {
  stable: 'יציב',
  too_easy: 'קל מדי',
  too_hard: 'קשה מדי',
  unstable: 'חוסר יציבות',
  discomfort: 'אי-נוחות',
  uncertain: 'לא ברור',
}

export const FEEDBACK_COLORS = {
  stable: 'var(--color-good)',
  too_easy: '#3b82f6',
  too_hard: '#f0a93a',
  unstable: '#f97316',
  discomfort: 'var(--color-bad)',
  uncertain: 'var(--color-text-muted)',
}

export function shoeStateLabel(state) {
  return SHOE_STATE_LABELS[state] || state
}

export function feedbackLabel(category) {
  return FEEDBACK_LABELS[category] || category
}

export function feedbackColor(category) {
  return FEEDBACK_COLORS[category] || 'var(--color-text-muted)'
}
