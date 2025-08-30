import { useState, useEffect, useRef, useMemo } from 'react'
import { getClassifications, getLatestClassification, getCurrentImageInfo } from '../api/client'
import type { Classification, CurrentImageInfo } from '../types'

interface UseClassificationsReturn {
  classifications: Classification[]
  latest: Classification | null
  currentImage: CurrentImageInfo | null
  loading: boolean
  error: string | null
  lastUpdate: Date | null
  refetch: () => Promise<void>
}

export const useClassifications = (): UseClassificationsReturn => {
  const [classifications, setClassifications] = useState<Classification[]>([])
  const [latest, setLatest] = useState<Classification | null>(null)
  const [currentImage, setCurrentImage] = useState<CurrentImageInfo | null>(null)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)
  const [lastUpdate, setLastUpdate] = useState<Date | null>(null)
  const abortControllerRef = useRef<AbortController | null>(null)
  const lastLatestId = useRef<number | null>(null)
  const pollIntervalRef = useRef<ReturnType<typeof setInterval> | null>(null)

  const fetchFullData = async () => {
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
      
      // Update the last seen ID
      if (latestData) {
        lastLatestId.current = latestData.id
      }
    } catch (err) {
      if (err instanceof Error && err.name !== 'AbortError') {
        setError(err.message)
      }
    } finally {
      setLoading(false)
    }
  }

  const checkForNewClassification = async () => {
    try {
      // Only check for latest classification to see if there's something new
      const latestData = await getLatestClassification()
      
      // If we have a new classification (different ID), fetch all data
      if (latestData && latestData.id !== lastLatestId.current) {
        console.log(`[Classifications] New classification detected (ID: ${latestData.id}), updating data...`)
        await fetchFullData()
      }
    } catch (err) {
      // Don't update error state for lightweight polls, just log
      console.warn('[Classifications] Check failed:', err)
    }
  }

  // Start polling for new classifications
  const startPolling = () => {
    if (pollIntervalRef.current) {
      clearInterval(pollIntervalRef.current)
    }
    
    // Poll every 2 seconds for new classifications
    pollIntervalRef.current = setInterval(checkForNewClassification, 2000)
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
    fetchFullData().then(() => {
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

  return { classifications, latest, currentImage, loading, error, lastUpdate, refetch: fetchFullData }
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
