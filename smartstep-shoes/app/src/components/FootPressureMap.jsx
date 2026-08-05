import { useSettings } from '../hooks/useSettings.js'
import { alertColor, pressureColor, pressureRadius } from '../utils/pressure.js'

const FOOT_PATH =
  'M50,8 C25,8 12,35 14,68 C16,95 28,105 22,135 C17,165 22,192 50,192 ' +
  'C78,192 83,165 78,135 C72,105 84,95 86,68 C88,35 75,8 50,8 Z'

const BALL1_POS = { x: 32, y: 55 }
const BALL5_POS = { x: 68, y: 55 }
const HEEL_POS = { x: 50, y: 158 }

function Foot({ side, data, pronationWarn, pronationAlert }) {
  const heel = data?.heel ?? 0
  const ball1 = data?.ball1 ?? 0
  const ball5 = data?.ball5 ?? 0
  const cop = data?.centerOfPressure ?? 0
  const pronation = data?.pronationPercent ?? 0

  const copX = 50 - cop * 18
  const copColor = alertColor(Math.abs(pronation), pronationWarn, pronationAlert)
  const mirror = side === 'left' ? 'scale(-1,1) translate(-100,0)' : undefined

  return (
    <svg viewBox="0 0 100 200" className="foot-svg" role="img" aria-label={side === 'left' ? 'רגל שמאל' : 'רגל ימין'}>
      <g transform={mirror}>
        <path d={FOOT_PATH} className="foot-outline" />
        <circle cx={HEEL_POS.x} cy={HEEL_POS.y} r={pressureRadius(heel)} fill={pressureColor(heel)} className="pressure-zone" />
        <circle cx={BALL1_POS.x} cy={BALL1_POS.y} r={pressureRadius(ball1)} fill={pressureColor(ball1)} className="pressure-zone" />
        <circle cx={BALL5_POS.x} cy={BALL5_POS.y} r={pressureRadius(ball5)} fill={pressureColor(ball5)} className="pressure-zone" />
        <circle cx={copX} cy={60} r="5" fill={copColor} stroke="white" strokeWidth="1.5" />
      </g>
      <text x="50" y="212" className="foot-label" textAnchor="middle">
        {side === 'left' ? 'שמאל' : 'ימין'}
      </text>
    </svg>
  )
}

export default function FootPressureMap({ liveData }) {
  const { settings } = useSettings()

  if (!liveData) return null

  const asymmetry = Math.abs(liveData.asymmetryPercent ?? 0)
  const barColor = alertColor(asymmetry, settings.thresholdAsymmetryWarn, settings.thresholdAsymmetryAlert)
  const needlePercent = Math.max(-50, Math.min(50, liveData.asymmetryPercent ?? 0))

  return (
    <div className="pressure-map-card">
      <div className="pressure-map-header">
        <span className="pressure-map-title">לחץ בזמן אמת</span>
      </div>

      <div className="pressure-map-feet" dir="ltr">
        <Foot
          side="left"
          data={liveData.left}
          pronationWarn={settings.thresholdPronationWarn}
          pronationAlert={settings.thresholdPronationAlert}
        />
        <Foot
          side="right"
          data={liveData.right}
          pronationWarn={settings.thresholdPronationWarn}
          pronationAlert={settings.thresholdPronationAlert}
        />
      </div>

      <div className="symmetry-bar-wrapper" dir="ltr">
        <span className="symmetry-label">שמאל</span>
        <div className="symmetry-bar">
          <div className="symmetry-bar-center" />
          <div
            className="symmetry-bar-needle"
            style={{ right: `calc(50% - ${needlePercent}%)`, background: barColor }}
          />
        </div>
        <span className="symmetry-label">ימין</span>
      </div>
    </div>
  )
}
