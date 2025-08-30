import { useState, useEffect, useRef } from 'react'
import { getHealth } from '../api/client'
import type { SystemHealth } from '../types'
import { useRealTime } from './useRealTime'

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

  // Use real-time hook for polling
  useRealTime(fetchHealth)

  // Initial fetch
  useEffect(() => {
    fetchHealth()

    // Cleanup on unmount
    return () => {
      if (abortControllerRef.current) {
        abortControllerRef.current.abort()
      }
    }
  }, [])

  return { health, loading, error, lastUpdate }
}
