import { useEffect, useRef, useState } from 'react'
import { getDocument, setDocument } from '../firestoreRest.js'

const DEFAULT_SETTINGS = {
  thresholdPronationWarn: 15,
  thresholdPronationAlert: 30,
  thresholdAsymmetryWarn: 15,
  thresholdAsymmetryAlert: 30,
  thresholdPressureActive: 500,
}

const DEBOUNCE_MS = 400
const POLL_MS = 3000

export function useSettings() {
  const [settings, setSettings] = useState(DEFAULT_SETTINGS)
  const [loading, setLoading] = useState(true)
  const pendingWrites = useRef({})
  const debounceTimer = useRef(null)

  useEffect(() => {
    let cancelled = false

    async function fetchSettings() {
      try {
        const value = await getDocument('state/settings')
        if (cancelled) return
        if (value) setSettings((prev) => ({ ...prev, ...value }))
      } finally {
        if (!cancelled) setLoading(false)
      }
    }

    fetchSettings()
    const intervalId = setInterval(fetchSettings, POLL_MS)
    return () => {
      cancelled = true
      clearInterval(intervalId)
    }
  }, [])

  function setSetting(key, value) {
    setSettings((prev) => ({ ...prev, [key]: value }))
    pendingWrites.current[key] = value

    if (debounceTimer.current) clearTimeout(debounceTimer.current)
    debounceTimer.current = setTimeout(() => {
      const updates = pendingWrites.current
      pendingWrites.current = {}
      setDocument('state/settings', updates)
    }, DEBOUNCE_MS)
  }

  return { settings, loading, setSetting }
}
