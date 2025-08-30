import { useState, useEffect, useRef } from 'react'
import { getHealth } from '../api/client'
import type { SystemHealth } from '../types'

interface UseSystemHealthReturn {
  health: SystemHealth | null
  loading: boolean
  error: string | null
  lastUpdate: Date | null
}

export const useSystemHealth = (): UseSystemHealthReturn => {
  const [health, setHealth] = useState<SystemHealth | null>(null)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)
  const [lastUpdate, setLastUpdate] = useState<Date | null>(null)
  const abortControllerRef = useRef<AbortController | null>(null)
  const pollIntervalRef = useRef<ReturnType<typeof setInterval> | null>(null)

  const fetchHealth = async () => {
    try {
      // Abort previous request
      if (abortControllerRef.current) {
        abortControllerRef.current.abort()
      }

      abortControllerRef.current = new AbortController()
      
      const healthData = await getHealth(abortControllerRef.current.signal)
      setHealth(healthData)
      setError(null)
      setLastUpdate(new Date())
    } catch (err) {
      if (err instanceof Error && err.name !== 'AbortError') {
        setError(err.message)
      }
    } finally {
      setLoading(false)
    }
  }

  // Start polling for system health (less frequent since it's just status)
  const startPolling = () => {
    if (pollIntervalRef.current) {
      clearInterval(pollIntervalRef.current)
    }
    
    // Poll every 5 seconds for system health
    pollIntervalRef.current = setInterval(fetchHealth, 5000)
  }

  // Stop polling
  const stopPolling = () => {
    if (pollIntervalRef.current) {
      clearInterval(pollIntervalRef.current)
      pollIntervalRef.current = null
    }
  }

  // Initial fetch and start polling
  useEffect(() => {
    fetchHealth().then(() => {
      startPolling()
    })

    // Cleanup on unmount
    return () => {
      stopPolling()
      if (abortControllerRef.current) {
        abortControllerRef.current.abort()
      }
    }
  }, [])

  return { health, loading, error, lastUpdate }
}
