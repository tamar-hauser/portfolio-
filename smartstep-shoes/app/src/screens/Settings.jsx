import { useSettings } from '../hooks/useSettings.js'

const SLIDERS = [
  {
    key: 'thresholdPronationWarn',
    label: 'אזהרת פרונציה',
    hint: 'אחוז סטייה שממנו מוצגת אזהרה ראשונה',
    min: 0,
    max: 100,
    unit: '%',
  },
  {
    key: 'thresholdPronationAlert',
    label: 'התראת פרונציה',
    hint: 'אחוז סטייה שממנו מוצגת התראה חמורה',
    min: 0,
    max: 100,
    unit: '%',
  },
  {
    key: 'thresholdAsymmetryWarn',
    label: 'אזהרת א-סימטריה',
    hint: 'אחוז הפרש בין הרגליים שממנו מוצגת אזהרה ראשונה',
    min: 0,
    max: 100,
    unit: '%',
  },
  {
    key: 'thresholdAsymmetryAlert',
    label: 'התראת א-סימטריה',
    hint: 'אחוז הפרש בין הרגליים שממנו מוצגת התראה חמורה',
    min: 0,
    max: 100,
    unit: '%',
  },
  {
    key: 'thresholdPressureActive',
    label: 'סף לחץ פעיל',
    hint: 'ערך ADC גולמי שמעליו נחשב שהמתאמנת עומדת על הנעל',
    min: 0,
    max: 4095,
    unit: '',
  },
]

export default function Settings() {
  const { settings, loading, setSetting } = useSettings()

  if (loading) {
    return <div className="screen"><p className="muted">טוען...</p></div>
  }

  return (
    <div className="screen">
      <h2 className="screen-title">הגדרות</h2>
      <p className="settings-note">שינוי כל סליידר נשמר אוטומטית ל-Firestore</p>

      <div className="settings-list">
        {SLIDERS.map((slider) => (
          <div key={slider.key} className="setting-card">
            <div className="setting-header">
              <label htmlFor={slider.key} className="setting-label">{slider.label}</label>
              <span className="setting-value">
                {settings[slider.key]}
                {slider.unit}
              </span>
            </div>
            <p className="setting-hint">{slider.hint}</p>
            <input
              id={slider.key}
              type="range"
              min={slider.min}
              max={slider.max}
              value={settings[slider.key]}
              onChange={(e) => setSetting(slider.key, Number(e.target.value))}
              className="setting-slider"
            />
          </div>
        ))}
      </div>
    </div>
  )
}
