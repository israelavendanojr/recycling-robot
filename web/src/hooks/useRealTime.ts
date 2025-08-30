import { useEffect, useRef, useState } from 'react'

// Global real-time tick every 1.5 seconds
let globalInterval: ReturnType<typeof setInterval> | null = null
let subscribers: Set<() => void> = new Set()

const POLL_INTERVAL = 1500

export const useRealTime = (callback?: () => void) => {
  const [tick, setTick] = useState(0)
  const callbackRef = useRef(callback)

  // Update callback ref
  useEffect(() => {
    callbackRef.current = callback
  }, [callback])

  useEffect(() => {
    const subscriber = () => {
      setTick(prev => prev + 1)
      if (callbackRef.current) {
        callbackRef.current()
      }
    }

    // Add subscriber
    subscribers.add(subscriber)

    // Start global interval if not already running
    if (!globalInterval) {
      globalInterval = setInterval(() => {
        subscribers.forEach(sub => sub())
      }, POLL_INTERVAL)
    }

    // Cleanup
    return () => {
      subscribers.delete(subscriber)
      
      // Stop global interval if no more subscribers
      if (subscribers.size === 0 && globalInterval) {
        clearInterval(globalInterval)
        globalInterval = null
      }
    }
  }, [])

  return { tick, lastUpdate: new Date() }
}
