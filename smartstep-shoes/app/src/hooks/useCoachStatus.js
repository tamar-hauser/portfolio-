import { useEffect, useState } from 'react'
import { getDocument } from '../firestoreRest.js'

const POLL_MS = 2000

export function useCoachStatus() {
  const [status, setStatus] = useState(null)
  const [loading, setLoading] = useState(true)

  useEffect(() => {
    let cancelled = false

    async function fetchStatus() {
      try {
        const value = await getDocument('state/coachStatus')
        if (cancelled) return
        setStatus(value)
      } finally {
        if (!cancelled) setLoading(false)
      }
    }

    fetchStatus()
    const intervalId = setInterval(fetchStatus, POLL_MS)
    return () => {
      cancelled = true
      clearInterval(intervalId)
    }
  }, [])

  return { status, loading }
}
