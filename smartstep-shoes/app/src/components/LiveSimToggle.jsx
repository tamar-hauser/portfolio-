import { useEffect, useRef, useState } from 'react'
import { startLiveSimulation, stopLiveSimulation } from '../utils/liveSim.js'

export default function LiveSimToggle() {
  const [active, setActive] = useState(false)
  const intervalIdRef = useRef(null)

  useEffect(() => {
    return () => {
      if (intervalIdRef.current) stopLiveSimulation(intervalIdRef.current)
    }
  }, [])

  function toggle() {
    if (active) {
      stopLiveSimulation(intervalIdRef.current)
      intervalIdRef.current = null
      setActive(false)
    } else {
      intervalIdRef.current = startLiveSimulation()
      setActive(true)
    }
  }

  return (
    <button type="button" className={'live-sim-toggle' + (active ? ' active' : '')} onClick={toggle}>
      <span className="live-sim-dot" />
      {active ? 'סימולציה חיה פעילה' : 'הפעל סימולציה חיה'}
    </button>
  )
}
