export function pressureColor(value) {
  const v = Math.max(0, Math.min(1, value))
  const hue = 220 - v * 220
  const saturation = 75
  const lightness = 55 - v * 12
  return `hsl(${hue}, ${saturation}%, ${lightness}%)`
}

export function pressureRadius(value, base = 15, max = 26) {
  const v = Math.max(0, Math.min(1, value))
  return base + v * (max - base)
}

export function alertColor(absValue, warnThreshold, alertThreshold) {
  if (absValue >= alertThreshold) return 'var(--color-bad)'
  if (absValue >= warnThreshold) return 'var(--color-warn)'
  return 'var(--color-good)'
}
