import React, { useState, useEffect, useRef } from 'react'
import { BarChart, Bar, XAxis, YAxis, ResponsiveContainer, Cell } from 'recharts'
import { getCounters } from '../api/client'
import { ErrorBanner } from './ErrorBanner'
import type { MaterialCounts } from '../types'
import { BAR_PALETTE, AXIS_TICK } from '../theme/charts'

interface MaterialCountData {
  material: string
  count: number
  color: string
}

const CustomLegend: React.FC<{ data: MaterialCountData[] }> = ({ data }) => (
  <div className="flex flex-wrap justify-center gap-4 mt-4">
    {data.map((item) => (
      <div key={item.material} className="flex items-center space-x-2">
        <div
          className="w-3 h-3 rounded-full"
          style={{ backgroundColor: item.color }}
        />
        <span className="text-sm text-brand-text-label capitalize">
          {item.material}
        </span>
      </div>
    ))}
  </div>
)



export const MaterialCounting: React.FC = () => {
  const [counts, setCounts] = useState<MaterialCounts | null>(null)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)
  const pollIntervalRef = useRef<ReturnType<typeof setInterval> | null>(null)

  const fetchCounts = async () => {
    try {
      const countersData = await getCounters()
      setCounts(countersData)
      setError(null)
    } catch (err) {
      if (err instanceof Error) {
        setError(err.message)
      }
    } finally {
      setLoading(false)
    }
  }

  // Start polling for material counts (slower since it changes less frequently)
  const startPolling = () => {
    if (pollIntervalRef.current) {
      clearInterval(pollIntervalRef.current)
    }
    
    // Poll every 10 seconds for material counts
    pollIntervalRef.current = setInterval(fetchCounts, 10000)
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
    fetchCounts().then(() => {
      startPolling()
    })

    // Cleanup on unmount
    return () => {
      stopPolling()
    }
  }, [])

  const chartData: MaterialCountData[] = React.useMemo(() => {
    if (!counts) return []
    
    return [
      { material: 'cardboard', count: counts.cardboard, color: BAR_PALETTE[0] },
      { material: 'glass', count: counts.glass, color: BAR_PALETTE[1] },
      { material: 'metal', count: counts.metal, color: BAR_PALETTE[2] },
      { material: 'plastic', count: counts.plastic, color: BAR_PALETTE[3] },
      { material: 'trash', count: counts.trash, color: BAR_PALETTE[4] },
    ]
  }, [counts])

  const totalItems = React.useMemo(() => {
    return chartData.reduce((sum, item) => sum + item.count, 0)
  }, [chartData])

  if (loading) {
    return (
      <div className="card">
        <div className="text-lg font-semibold text-brand-text-primary mb-6">
          Material Counting
        </div>
        <div className="h-64 bg-gray-200 rounded animate-pulse"></div>
      </div>
    )
  }

  if (error) {
    return (
      <ErrorBanner
        title="Material Counting Error"
        error={error}
      />
    )
  }

  return (
    <div className="card">
      <div className="flex justify-between items-center mb-6">
        <div className="text-lg font-semibold text-brand-text-primary">
          Material Counting
        </div>
        <div className="text-right">
          <div className="text-2xl font-bold text-brand-teal-medium">
            {totalItems}
          </div>
          <div className="text-sm text-brand-text-label">Total Items</div>
        </div>
      </div>

      <div className="h-64">
        <ResponsiveContainer width="100%" height="100%">
          <BarChart
            data={chartData}
            margin={{ top: 20, right: 30, left: 20, bottom: 30 }}
          >
            {/* Materials along bottom */}
            <XAxis
              dataKey="material"
              axisLine={false}
              tickLine={false}
              tick={{ fontSize: 12, fill: AXIS_TICK }}
            />

            {/* Counts up the side */}
            <YAxis
              type="number"
              axisLine={false}
              tickLine={false}
              tick={{ fontSize: 12, fill: AXIS_TICK }}
            />

            <Bar
              dataKey="count"
              radius={[4, 4, 0, 0]} // rounded top corners
              label={{ position: 'top', fill: AXIS_TICK, fontSize: 12, fontWeight: 500 }}
            >
              {chartData.map((entry, index) => (
                <Cell key={`cell-${index}`} fill={entry.color} />
              ))}
            </Bar>
          </BarChart>
        </ResponsiveContainer>
      </div>


      <CustomLegend data={chartData} />
    </div>
  )
}
