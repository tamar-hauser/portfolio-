import { useEffect, useState } from 'react'
import { listCollection } from '../firestoreRest.js'

const POLL_MS = 3000

export function useSessions() {
  const [sessions, setSessions] = useState([])
  const [loading, setLoading] = useState(true)

  useEffect(() => {
    let cancelled = false

    async function fetchSessions() {
      try {
        const list = await listCollection('sessions')
        if (cancelled) return
        list.sort((a, b) => b.timestamp - a.timestamp)
        setSessions(list)
      } finally {
        if (!cancelled) setLoading(false)
      }
    }

    fetchSessions()
    const intervalId = setInterval(fetchSessions, POLL_MS)
    return () => {
      cancelled = true
      clearInterval(intervalId)
    }
  }, [])

  return { sessions, loading }
}
