import { useState, useEffect, useRef, useMemo } from 'react'
import { getClassifications, getLatestClassification, getCurrentImageInfo } from '../api/client'
import type { Classification, CurrentImageInfo } from '../types'
import { useRealTime } from './useRealTime'

interface UseClassificationsReturn {
  classifications: Classification[]
  latest: Classification | null
  currentImage: CurrentImageInfo | null
  loading: boolean
  error: string | null
  lastUpdate: Date | null
}

export const useClassifications = (): UseClassificationsReturn => {
  const [classifications, setClassifications] = useState<Classification[]>([])
  const [latest, setLatest] = useState<Classification | null>(null)
  const [currentImage, setCurrentImage] = useState<CurrentImageInfo | null>(null)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)
  const [lastUpdate, setLastUpdate] = useState<Date | null>(null)
  const abortControllerRef = useRef<AbortController | null>(null)

  const fetchData = async () => {
    try {
      // Abort previous request
      if (abortControllerRef.current) {
        abortControllerRef.current.abort()
      }

      abortControllerRef.current = new AbortController()
      
      // Fetch all data in parallel
      const [classificationsData, latestData, imageData] = await Promise.all([
        getClassifications(abortControllerRef.current.signal),
        getLatestClassification(abortControllerRef.current.signal),
        getCurrentImageInfo(abortControllerRef.current.signal),
      ])

      setClassifications(classificationsData)
      setLatest(latestData)
      setCurrentImage(imageData)
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
  useRealTime(fetchData)

  // Initial fetch
  useEffect(() => {
    fetchData()

    // Cleanup on unmount
    return () => {
      if (abortControllerRef.current) {
        abortControllerRef.current.abort()
      }
    }
  }, [])

  return { classifications, latest, currentImage, loading, error, lastUpdate }
}

// Hook for recent N classifications for charts
export const useRecentClassifications = (count: number = 100): Classification[] => {
  const { classifications } = useClassifications()
  
  return useMemo(() => {
    return classifications
      .sort((a, b) => b.timestamp - a.timestamp)
      .slice(0, count)
      .reverse() // Reverse to get chronological order for charts
  }, [classifications, count])
}
