import { useState } from 'react'
import { Check } from 'lucide-react'
import { seedDemoSessions } from '../utils/demoData.js'

export default function DemoDataButton() {
  const [status, setStatus] = useState('idle') // idle | loading | done | error

  async function handleClick() {
    setStatus('loading')
    try {
      await seedDemoSessions()
      setStatus('done')
    } catch {
      setStatus('error')
    }
    setTimeout(() => setStatus('idle'), 2500)
  }

  return (
    <button type="button" className="demo-data-btn" onClick={handleClick} disabled={status === 'loading'}>
      {status === 'loading' && 'יוצר נתוני דמו...'}
      {status === 'done' && (
        <span style={{ display: 'inline-flex', alignItems: 'center', gap: 4 }}>
          נוצרו סשנים <Check size={14} />
        </span>
      )}
      {status === 'error' && 'שגיאה בכתיבה ל-Firebase'}
      {status === 'idle' && '+ נתוני דמו'}
    </button>
  )
}
