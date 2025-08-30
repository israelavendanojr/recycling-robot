import axios, { AxiosResponse } from 'axios'
import { z } from 'zod'
import type { SystemHealth, Classification, MaterialCounts, CurrentImageInfo } from '../types'

// Base URL configuration
const getBaseURL = (): string => {
  const envUrl = import.meta.env.VITE_API_BASE_URL
  if (envUrl) return envUrl
  
  // Auto-detect based on current location
  if (typeof window !== 'undefined') {
    const isLocalhost = window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1'
    return isLocalhost ? 'http://localhost:8000' : 'http://backend:8000'
  }
  
  return 'http://backend:8000'
}

// Create axios instance
const api = axios.create({
  baseURL: getBaseURL(),
  timeout: 10000,
  headers: {
    'Content-Type': 'application/json',
  },
})

// Retry utility
const withRetry = async <T>(
  fn: () => Promise<AxiosResponse<T>>,
  maxRetries = 2,
  delay = 1000
): Promise<T> => {
  let lastError: Error

  for (let attempt = 0; attempt <= maxRetries; attempt++) {
    try {
      const response = await fn()
      return response.data
    } catch (error) {
      lastError = error as Error
      
      if (attempt < maxRetries) {
        await new Promise(resolve => setTimeout(resolve, delay * Math.pow(2, attempt)))
      }
    }
  }

  throw lastError!
}

// Zod schemas for runtime validation
const SystemHealthSchema = z.object({
  status: z.enum(['ok', 'error']),
  services: z.object({
    ros2: z.boolean(),
    camera: z.boolean(),
    db: z.boolean(),
  }),
  system: z.object({
    cpu_percent: z.number(),
    memory_percent: z.number(),
    disk_usage: z.number(),
    uptime: z.number(),
  }),
  timestamp: z.number(),
  classifier_running: z.boolean(),
})

const ClassificationSchema = z.object({
  id: z.number(),
  timestamp: z.number(),
  label: z.string(),
  confidence: z.number(),
  raw_logits: z.string().optional(),
  image_source: z.string(),
  created_at: z.string(),
})

const MaterialCountsSchema = z.object({
  cardboard: z.number(),
  glass: z.number(),
  metal: z.number(),
  plastic: z.number(),
  trash: z.number(),
})

const CurrentImageInfoSchema = z.object({
  ts: z.number(),
})

// API functions
export const getHealth = async (signal?: AbortSignal): Promise<SystemHealth> => {
  return withRetry(() => api.get('/api/health', { signal }))
    .then(data => SystemHealthSchema.parse(data))
}

export const getClassifications = async (signal?: AbortSignal): Promise<Classification[]> => {
  return withRetry(() => api.get('/api/classifications', { signal }))
    .then(data => z.array(ClassificationSchema).parse(data))
}

export const getLatestClassification = async (signal?: AbortSignal): Promise<Classification | null> => {
  try {
    const data = await withRetry(() => api.get('/api/classifications/latest', { signal }))
    return data ? ClassificationSchema.parse(data) : null
  } catch (error) {
    // Return null if no latest classification exists
    if (axios.isAxiosError(error) && error.response?.status === 404) {
      return null
    }
    throw error
  }
}

export const getCurrentImageInfo = async (signal?: AbortSignal): Promise<CurrentImageInfo> => {
  return withRetry(() => api.get('/api/current_image', { signal }))
    .then(data => CurrentImageInfoSchema.parse(data))
}

export const getCounters = async (signal?: AbortSignal): Promise<MaterialCounts> => {
  return withRetry(() => api.get('/api/counters', { signal }))
    .then(data => MaterialCountsSchema.parse(data))
}

// Utility to get current frame URL with timestamp
export const getCurrentFrameURL = (ts: number): string => {
  return `${getBaseURL()}/api/current_frame.jpg?ts=${ts}`
}

export default api
