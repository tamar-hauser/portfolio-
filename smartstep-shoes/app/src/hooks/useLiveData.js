import { useEffect, useState } from 'react'
import { getDocument } from '../firestoreRest.js'

const POLL_MS = 1000

export function useLiveData() {
  const [liveData, setLiveData] = useState(null)
  const [loading, setLoading] = useState(true)

  useEffect(() => {
    let cancelled = false

    async function fetchLiveData() {
      try {
        const value = await getDocument('state/liveData')
        if (cancelled) return
        setLiveData(value)
      } finally {
        if (!cancelled) setLoading(false)
      }
    }

    fetchLiveData()
    const intervalId = setInterval(fetchLiveData, POLL_MS)
    return () => {
      cancelled = true
      clearInterval(intervalId)
    }
  }, [])

  return { liveData, loading }
}
