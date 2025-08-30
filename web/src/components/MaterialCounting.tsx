import React, { useState, useEffect } from 'react'
import { BarChart, Bar, XAxis, YAxis, ResponsiveContainer, Cell } from 'recharts'
import { getCounters } from '../api/client'
import { useRealTime } from '../hooks/useRealTime'
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

const CustomBarLabel: React.FC<{ 
  x?: number; 
  y?: number; 
  width?: number; 
  value?: number;
}> = ({ x = 0, y = 0, width = 0, value = 0 }) => (
  <text
    x={x + width + 8}
    y={y + 12}
    fill={AXIS_TICK}
    fontSize={12}
    fontWeight="500"
  >
    {value}
  </text>
)

export const MaterialCounting: React.FC = () => {
  const [counts, setCounts] = useState<MaterialCounts | null>(null)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)

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

  // Use real-time polling
  useRealTime(fetchCounts)

  // Initial fetch
  useEffect(() => {
    fetchCounts()
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
      <div className="card bg-red-50 border-red-200">
        <div className="text-red-600 font-medium">Material Counting Error</div>
        <div className="text-red-500 text-sm mt-1">{error}</div>
      </div>
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
            layout="horizontal"
            margin={{ top: 20, right: 60, left: 80, bottom: 5 }}
          >
            <XAxis 
              type="number"
              axisLine={false}
              tickLine={false}
              tick={{ fontSize: 12, fill: AXIS_TICK }}
            />
            <YAxis 
              type="category"
              dataKey="material"
              axisLine={false}
              tickLine={false}
              tick={{ fontSize: 12, fill: AXIS_TICK }}
              width={70}
            />
            <Bar 
              dataKey="count" 
              radius={[0, 4, 4, 0]}
              label={<CustomBarLabel />}
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
