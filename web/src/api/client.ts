import axios from 'axios'
import { z } from 'zod'
import type { SystemHealth, Classification, MaterialCounts } from '../types'
import { devLog } from './devLog'

const baseURL =
  import.meta.env.VITE_API_BASE_URL ??
  (location.hostname === 'localhost' ? 'http://localhost:8000' : 'http://backend:8000')

export const http = axios.create({ baseURL, timeout: 5000 })

// ---------- Schemas ----------
const SystemHealthSchema: z.ZodType<SystemHealth> = z.object({
  status: z.union([z.literal('ok'), z.literal('error')]),
  services: z.object({
    ros2: z.boolean(),
    camera: z.boolean(),
    db: z.boolean(),
    api: z.boolean().optional(),
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

const toNum = z.string().transform((v) => Number(v))
const num = z.union([z.number(), toNum]).transform((v) => Number(v))

const ClassificationSchema = z.object({
  id: num,
  timestamp: num,              // allow string or number
  label: z.string(),
  confidence: num,
  raw_logits: z.union([z.string(), z.null()]).optional(),  // Allow null values
  image_source: z.string(),
  created_at: z.union([z.string(), num]).transform(val => String(val)),  // Convert numbers to strings
})

// /api/classifications returns: { classifications: [...], count: number, success: boolean }
const ClassificationsUnion = z.union([
  z.array(ClassificationSchema), // fallback for direct array
  z.object({ 
    classifications: z.array(ClassificationSchema),
    count: z.number(),
    success: z.boolean()
  }),
])

// latest returns: { classification: {...}, success: boolean }
const LatestUnion = z.union([
  ClassificationSchema, // fallback for direct object
  z.object({ 
    classification: ClassificationSchema,
    success: z.boolean()
  }),
  z.null(),
])

// current_image returns: { timestamp: number, image_url: string, source: string, success: boolean }
const CurrentImageUnion = z.union([
  z.object({ ts: num }), // fallback
  z.object({ timestamp: num }), // fallback
  z.object({ 
    timestamp: num,
    image_url: z.string(),
    source: z.string(),
    success: z.boolean()
  }),
])

const MaterialCountsSchema = z.object({
  cardboard: num,
  glass: num,
  metal: num,
  plastic: num,
  trash: num,
})

// ---------- Normalizers ----------
function normalizeClassifications(raw: unknown): Classification[] {
  const parsed = ClassificationsUnion.safeParse(raw)
  if (!parsed.success) throw parsed.error
  if (Array.isArray(parsed.data)) return parsed.data
  if ('classifications' in parsed.data) return parsed.data.classifications
  return []
}

function normalizeLatest(raw: unknown): Classification | null {
  const parsed = LatestUnion.safeParse(raw)
  if (!parsed.success) throw parsed.error
  if (parsed.data === null) return null
  if ('classification' in parsed.data) return parsed.data.classification
  return parsed.data
}

function normalizeTs(raw: unknown): number {
  const parsed = CurrentImageUnion.safeParse(raw)
  if (!parsed.success) throw parsed.error
  if ('ts' in parsed.data) return Number(parsed.data.ts)
  if ('timestamp' in parsed.data && typeof parsed.data.timestamp === 'number') {
    return Number(parsed.data.timestamp)
  }
  // For the real API shape, extract timestamp from image_url
  if ('image_url' in parsed.data && typeof parsed.data.image_url === 'string') {
    const match = parsed.data.image_url.match(/ts=(\d+)/)
    if (match) return Number(match[1])
  }
  return Number(parsed.data.timestamp)
}

// ---------- Typed API ----------
export async function getHealth(signal?: AbortSignal): Promise<SystemHealth> {
  const { data } = await http.get('/api/health', { signal })
  devLog('GET /api/health ->', data)
  return SystemHealthSchema.parse(data)
}

export async function getClassifications(signal?: AbortSignal): Promise<Classification[]> {
  const { data } = await http.get('/api/classifications', { signal })
  devLog('GET /api/classifications ->', data)
  return normalizeClassifications(data)
}

export async function getLatestClassification(signal?: AbortSignal): Promise<Classification | null> {
  try {
    const { data } = await http.get('/api/classifications/latest', { signal })
    devLog('GET /api/classifications/latest ->', data)
    return normalizeLatest(data)
  } catch (error) {
    // Return null if no latest classification exists or if aborted
    if (axios.isAxiosError(error) && (error.response?.status === 404 || error.name === 'AbortError')) {
      return null
    }
    throw error
  }
}

export async function getCurrentImageInfo(signal?: AbortSignal): Promise<{ ts: number }> {
  const { data } = await http.get('/api/current_image', { signal })
  devLog('GET /api/current_image ->', data)
  return { ts: normalizeTs(data) }
}

export async function getCounters(signal?: AbortSignal): Promise<MaterialCounts> {
  const { data } = await http.get('/api/counters', { signal })
  devLog('GET /api/counters ->', data)
  return MaterialCountsSchema.parse(data)
}

// Utility to get current frame URL with timestamp
export const getCurrentFrameURL = (ts: number): string => {
  return `${baseURL}/api/current_frame.jpg?ts=${ts}`
}

// Manual frame capture
export async function captureFrame(signal?: AbortSignal): Promise<{ status: string; message: string }> {
  const { data } = await http.post('/api/capture', {}, { signal })
  devLog('POST /api/capture ->', data)
  return data
}

// Quit pipeline
export async function quitPipeline(signal?: AbortSignal): Promise<{ status: string; message: string }> {
  const { data } = await http.post('/api/quit', {}, { signal })
  devLog('POST /api/quit ->', data)
  return data
}
