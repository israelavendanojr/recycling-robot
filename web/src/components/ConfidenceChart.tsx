import React from 'react'
import { LineChart, Line, XAxis, YAxis, ResponsiveContainer, Tooltip } from 'recharts'
import { format } from 'date-fns'
import { useRecentClassifications } from '../hooks/useClassifications'
import { LINE_COLOR, AXIS_TICK } from '../theme/charts'

interface ChartDataPoint {
  timestamp: number
  confidence: number
  label: string
  time: string
}

const CustomTooltip: React.FC<{
  active?: boolean;
  payload?: Array<{ payload: ChartDataPoint }>;
}> = ({ active, payload }) => {
  if (active && payload && payload.length) {
    const data = payload[0].payload as ChartDataPoint
    return (
      <div className="bg-white p-3 border border-brand-border rounded-lg shadow-md">
        <div className="text-sm font-medium text-brand-text-primary">
          {data.label}
        </div>
        <div className="text-sm text-brand-text-secondary">
          Confidence: {(data.confidence * 100).toFixed(1)}%
        </div>
        <div className="text-xs text-brand-text-label">
          {format(new Date(data.timestamp * 1000), 'HH:mm:ss')}
        </div>
      </div>
    )
  }
  return null
}

const CustomLegend: React.FC = () => (
  <div className="flex justify-end mt-2">
    <div className="flex items-center space-x-2">
      <div 
        className="w-3 h-0.5 rounded"
        style={{ backgroundColor: LINE_COLOR }}
      />
      <span className="text-sm text-brand-text-label">Confidence</span>
    </div>
  </div>
)

export const ConfidenceChart: React.FC = () => {
  const recentClassifications = useRecentClassifications(100)

  const chartData: ChartDataPoint[] = React.useMemo(() => {
    if (!recentClassifications || recentClassifications.length === 0) {
      return []
    }
    
    return recentClassifications
      .filter(classification => classification && typeof classification.confidence === 'number')
      .map(classification => ({
        timestamp: classification.timestamp,
        confidence: classification.confidence,
        label: classification.label,
        time: format(new Date(classification.timestamp * 1000), 'HH:mm'),
      }))
  }, [recentClassifications])

  if (chartData.length === 0) {
    return (
      <div className="card">
        <div className="text-lg font-semibold text-brand-text-primary mb-6">
          Confidence Trends
        </div>
        <div className="h-64 flex items-center justify-center text-brand-text-secondary" role="status">
          <div className="text-center">
            <div className="text-lg mb-2">No confidence data available</div>
            <div className="text-sm">Charts will appear after classifications are recorded</div>
          </div>
        </div>
      </div>
    )
  }

  return (
    <div className="card">
      <div className="text-lg font-semibold text-brand-text-primary mb-6">
        Confidence Trends
      </div>

      <div className="h-64">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart
            data={chartData}
            margin={{ top: 20, right: 30, left: 20, bottom: 5 }}
          >
            <XAxis
              dataKey="time"
              axisLine={false}
              tickLine={false}
              tick={{ fontSize: 10, fill: AXIS_TICK }}
              interval="preserveStartEnd"
            />
            <YAxis
              domain={[0, 1]}
              axisLine={false}
              tickLine={false}
              tick={{ fontSize: 12, fill: AXIS_TICK }}
              tickFormatter={(value) => `${(value * 100).toFixed(0)}%`}
            />
            <Tooltip content={<CustomTooltip />} />
            <Line
              type="monotone"
              dataKey="confidence"
              stroke={LINE_COLOR}
              strokeWidth={2}
              dot={{ fill: LINE_COLOR, strokeWidth: 0, r: 3 }}
              activeDot={{ r: 5, fill: LINE_COLOR }}
            />
          </LineChart>
        </ResponsiveContainer>
      </div>

      <CustomLegend />
    </div>
  )
}
