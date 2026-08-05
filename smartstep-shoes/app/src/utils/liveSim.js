import { setDocument } from '../firestoreRest.js'

const TICK_MS = 500

function clamp01(v) {
  return Math.max(0, Math.min(1, v))
}

function walk(value, step) {
  return clamp01(value + (Math.random() - 0.5) * step)
}

function buildFootFrame(prev) {
  const heel = walk(prev?.heel ?? 0.5, 0.12)
  const ball1 = walk(prev?.ball1 ?? 0.45, 0.15)
  const ball5 = walk(prev?.ball5 ?? 0.45, 0.15)
  const ballSum = ball1 + ball5
  const pronationPercent = ballSum > 0.01 ? ((ball1 - ball5) / ballSum) * 100 : 0
  const centerOfPressure = ballSum > 0.01 ? (ball1 - ball5) / ballSum : 0

  return { heel, ball1, ball5, centerOfPressure, pronationPercent }
}

export function startLiveSimulation() {
  let left = buildFootFrame(null)
  let right = buildFootFrame(null)

  const intervalId = setInterval(() => {
    left = buildFootFrame(left)
    right = buildFootFrame(right)

    const leftTotal = (left.heel + left.ball1 + left.ball5) / 3
    const rightTotal = (right.heel + right.ball1 + right.ball5) / 3
    const maxTotal = Math.max(leftTotal, rightTotal, 0.01)
    const asymmetryPercent = ((rightTotal - leftTotal) / maxTotal) * 100

    const alertTriggered = Math.abs(left.pronationPercent) > 30 || Math.abs(right.pronationPercent) > 30

    setDocument('state/liveData', {
      state: alertTriggered ? 'alert' : 'training',
      left,
      right,
      asymmetryPercent,
      timestamp: Date.now(),
    })
  }, TICK_MS)

  return intervalId
}

export function stopLiveSimulation(intervalId) {
  clearInterval(intervalId)
}
