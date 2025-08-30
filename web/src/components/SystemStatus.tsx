import React from 'react'
import { format } from 'date-fns'
import { useSystemHealth } from '../hooks/useSystemHealth'
import { ErrorBanner } from './ErrorBanner'

const ServiceIndicator: React.FC<{ name: string; status: boolean }> = ({ name, status }) => (
  <div className="flex items-center space-x-2">
    <div
      className={`w-3 h-3 rounded-full ${
        status ? 'bg-brand-teal-medium' : 'bg-gray-400'
      }`}
      aria-label={`${name} status: ${status ? 'online' : 'offline'}`}
    />
    <span className="text-sm font-medium text-brand-text-primary">{name}</span>
  </div>
)

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
    <div className="text-xs text-brand-text-label">{label}</div>
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

export const SystemStatus: React.FC = () => {
  const { health, loading, error, lastUpdate } = useSystemHealth()

  if (loading) {
    return (
      <div className="grid grid-cols-1 md:grid-cols-4 gap-6">
        {[...Array(4)].map((_, i) => (
          <div key={i} className="card animate-pulse">
            <div className="h-4 bg-gray-200 rounded mb-2"></div>
            <div className="h-3 bg-gray-200 rounded w-2/3"></div>
          </div>
        ))}
      </div>
    )
  }

  if (error) {
    return (
      <ErrorBanner
        title="System Status Error"
        error={error}
      />
    )
  }

  if (!health) return null

  return (
    <div role="status" aria-label="System status">
      <div className="grid grid-cols-1 md:grid-cols-4 gap-6">
        {/* ROS2 Service */}
        <div className="card">
          <div className="text-sm font-medium text-brand-text-label mb-3">ROS2</div>
          <ServiceIndicator name="Service" status={health.services.ros2} />
        </div>

        {/* Camera Service */}
        <div className="card">
          <div className="text-sm font-medium text-brand-text-label mb-3">Camera</div>
          <ServiceIndicator name="Feed" status={health.services.camera} />
        </div>

        {/* Database Service */}
        <div className="card">
          <div className="text-sm font-medium text-brand-text-label mb-3">Database</div>
          <ServiceIndicator name="Connection" status={health.services.db} />
        </div>

        {/* System Metrics */}
        <div className="card">
          <div className="text-sm font-medium text-brand-text-label mb-3">Metrics</div>
          <div className="grid grid-cols-2 gap-2 text-xs">
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
        </div>
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
