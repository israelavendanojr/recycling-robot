import React, { useMemo } from 'react'
import { format } from 'date-fns'
import { PieChart, Pie, Cell, ResponsiveContainer } from 'recharts'
import { useClassifications } from '../hooks/useClassifications'
import { getCurrentFrameURL } from '../api/client'
import { COLORS } from '../theme/tokens'

const DonutChart: React.FC<{ confidence: number }> = ({ confidence }) => {
  const data = [
    { name: 'Confidence', value: confidence, color: COLORS.teal.medium },
    { name: 'Uncertainty', value: 100 - confidence, color: COLORS.bg.secondary },
  ]

  return (
    <div className="relative w-32 h-32">
      <ResponsiveContainer width="100%" height="100%">
        <PieChart>
          <Pie
            data={data}
            cx="50%"
            cy="50%"
            innerRadius={30}
            outerRadius={50}
            dataKey="value"
            strokeWidth={0}
          >
            {data.map((entry, index) => (
              <Cell key={`cell-${index}`} fill={entry.color} />
            ))}
          </Pie>
        </PieChart>
      </ResponsiveContainer>
      <div className="absolute inset-0 flex items-center justify-center">
        <div className="text-center">
          <div className="text-lg font-bold text-brand-text-primary">
            {confidence.toFixed(1)}%
          </div>
        </div>
      </div>
    </div>
  )
}

export const ClassificationPanel: React.FC = () => {
  const { latest, currentImage, loading, error } = useClassifications()

  const imageUrl = useMemo(() => {
    return currentImage ? getCurrentFrameURL(currentImage.ts) : null
  }, [currentImage])

  if (loading) {
    return (
      <div className="card">
        <div className="text-lg font-semibold text-brand-text-primary mb-6">
          Current Classification
        </div>
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
          {/* Image skeleton */}
          <div className="aspect-video bg-gray-200 rounded-lg animate-pulse border border-brand-border"></div>
          
          {/* Details skeleton */}
          <div className="space-y-4">
            <div className="h-8 bg-gray-200 rounded animate-pulse"></div>
            <div className="h-6 bg-gray-200 rounded w-3/4 animate-pulse"></div>
            <div className="w-32 h-32 bg-gray-200 rounded-full animate-pulse mx-auto"></div>
          </div>
        </div>
      </div>
    )
  }

  if (error) {
    return (
      <div className="card bg-red-50 border-red-200">
        <div className="text-red-600 font-medium">Classification Error</div>
        <div className="text-red-500 text-sm mt-1">{error}</div>
      </div>
    )
  }

  if (!latest || !currentImage) {
    return (
      <div className="card">
        <div className="text-lg font-semibold text-brand-text-primary mb-6">
          Current Classification
        </div>
        <div className="text-center text-brand-text-secondary py-8">
          No classifications available
        </div>
      </div>
    )
  }

  return (
    <div className="card">
      <div className="text-lg font-semibold text-brand-text-primary mb-6">
        Current Classification
      </div>
      
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* Image */}
        <div className="border border-brand-border rounded-lg overflow-hidden">
          {imageUrl ? (
            <img
              src={imageUrl}
              alt="Current classification frame"
              className="w-full h-auto aspect-video object-cover"
              key={currentImage.ts} // Force re-render when timestamp changes
            />
          ) : (
            <div className="aspect-video bg-gray-100 flex items-center justify-center">
              <span className="text-brand-text-secondary">No image available</span>
            </div>
          )}
        </div>

        {/* Classification Details */}
        <div className="space-y-4">
          {/* Label */}
          <div className="text-center">
            <div className="text-3xl font-bold text-brand-text-primary uppercase tracking-wider">
              {latest.label}
            </div>
          </div>

          {/* Confidence */}
          <div className="text-center">
            <div className="text-4xl font-bold text-brand-teal-medium mb-2">
              {(latest.confidence * 100).toFixed(1)}%
            </div>
            <div className="text-sm text-brand-text-label">Confidence</div>
          </div>

          {/* Donut Chart */}
          <div className="flex justify-center">
            <DonutChart confidence={latest.confidence * 100} />
          </div>

          {/* Metadata */}
          <div className="space-y-2 text-sm">
            <div className="flex justify-between">
              <span className="text-brand-text-label">Timestamp:</span>
              <span className="text-brand-text-primary">
                {format(new Date(latest.timestamp * 1000), 'HH:mm:ss')}
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-brand-text-label">Source:</span>
              <span className="text-brand-text-primary">{latest.image_source}</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}
