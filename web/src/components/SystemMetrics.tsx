import React from 'react'
import { format } from 'date-fns'
import { useSystemHealth } from '../hooks/useSystemHealth'
import { ErrorBanner } from './ErrorBanner'

const MetricCard: React.FC<{ label: string; value: string; unit?: string }> = ({ 
  label, 
  value, 
  unit 
}) => (
  <div className="text-center">
    <div className="text-lg font-semibold text-brand-text-primary">
      {value}
      {unit && <span className="text-sm text-brand-text-secondary">{unit}</span>}
    </div>
    <div className="text-sm text-brand-text-label">{label}</div>
  </div>
)

const formatUptime = (seconds: number): string => {
  const days = Math.floor(seconds / 86400)
  const hours = Math.floor((seconds % 86400) / 3600)
  const minutes = Math.floor((seconds % 3600) / 60)
  
  if (days > 0) return `${days}d ${hours}h`
  if (hours > 0) return `${hours}h ${minutes}m`
  return `${minutes}m`
}

export const SystemMetrics: React.FC = () => {
  const { health, loading, error, lastUpdate } = useSystemHealth()

  if (loading) {
    return (
      <div className="card animate-pulse">
        <div className="h-4 bg-gray-200 rounded mb-4 w-1/4"></div>
        <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
          {[...Array(4)].map((_, i) => (
            <div key={i} className="text-center">
              <div className="h-6 bg-gray-200 rounded mb-1"></div>
              <div className="h-3 bg-gray-200 rounded"></div>
            </div>
          ))}
        </div>
      </div>
    )
  }

  if (error) {
    return (
      <ErrorBanner
        title="System Metrics Error"
        error={error}
      />
    )
  }

  if (!health) return null

  return (
    <div className="card">
      <div className="text-sm font-medium text-brand-text-label mb-4">System Metrics</div>
      <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
        <MetricCard 
          label="CPU" 
          value={health.system.cpu_percent.toFixed(1)} 
          unit="%" 
        />
        <MetricCard 
          label="Memory" 
          value={health.system.memory_percent.toFixed(1)} 
          unit="%" 
        />
        <MetricCard 
          label="Disk" 
          value={health.system.disk_usage.toFixed(1)} 
          unit="%" 
        />
        <MetricCard 
          label="Uptime" 
          value={formatUptime(health.system.uptime)} 
        />
      </div>
      
      {/* Last Updated */}
      {lastUpdate && (
        <div className="mt-4 text-xs text-brand-text-secondary text-center">
          Last updated: {format(lastUpdate, 'HH:mm:ss')}
        </div>
      )}
    </div>
  )
}
