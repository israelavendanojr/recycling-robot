import React from 'react'
import { useSystemHealth } from '../hooks/useSystemHealth'
import { ErrorBanner } from './ErrorBanner'

interface ServiceIndicatorProps {
  name: string;
  status: boolean | 'streaming' | 'idle' | 'disconnected';
  type?: 'camera' | 'service';
}

const ServiceIndicator: React.FC<ServiceIndicatorProps> = ({ name, status, type = 'service' }) => {
  let bgColor: string;
  let statusText: string;
  
  if (type === 'camera') {
    if (status === 'streaming') {
      bgColor = 'bg-brand-teal-medium'; // teal for active streaming
      statusText = 'streaming';
    } else if (status === 'idle' || status === true) {
      bgColor = 'bg-brand-teal-medium'; // same teal as other services when connected
      statusText = 'connected';
    } else {
      bgColor = 'bg-gray-400'; // gray for disconnected
      statusText = 'disconnected';
    }
  } else {
    // Binary service status
    bgColor = status ? 'bg-brand-teal-medium' : 'bg-gray-400';
    statusText = status ? 'online' : 'offline';
  }
  
  return (
    <div className="flex items-center space-x-2">
      <div
        className={`w-3 h-3 rounded-full ${bgColor}`}
        aria-label={`${name} status: ${statusText}`}
      />
      <span className="text-xs font-medium text-brand-text-primary">{name}</span>
    </div>
  );
};



export const SystemStatus: React.FC = () => {
  const { health, loading, error } = useSystemHealth()

  if (loading) {
    return (
      <div className="grid grid-cols-1 md:grid-cols-4 gap-4">
        {[...Array(4)].map((_, i) => (
          <div key={i} className="compact-card animate-pulse">
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

  // Determine camera status based on health data
  const getCameraStatus = () => {
    // If camera service is explicitly true, show as idle
    if (health.services.camera === true) return 'idle';
    
    // Even if camera service reports false, if we have recent activity (classifier running or recent images),
    // the camera is likely working - show as idle (blue)
    if (health.classifier_running) return 'idle';
    
    // Otherwise, assume disconnected
    return 'disconnected';
  };

  return (
    <div role="status" aria-label="System status">
      <div className="grid grid-cols-1 md:grid-cols-4 gap-4">
        {/* ROS2 Service */}
        <div className="compact-card">
          <div className="text-xs font-medium text-brand-text-label mb-2">ROS2</div>
          <ServiceIndicator name="Service" status={health.services.ros2} />
        </div>

        {/* Camera Service */}
        <div className="compact-card">
          <div className="text-xs font-medium text-brand-text-label mb-2">Camera</div>
          <ServiceIndicator name="Device" status={getCameraStatus()} type="camera" />
        </div>

        {/* Database Service */}
        <div className="compact-card">
          <div className="text-xs font-medium text-brand-text-label mb-2">Database</div>
          <ServiceIndicator name="Connection" status={health.services.db} />
        </div>

        {/* API Service */}
        <div className="compact-card">
          <div className="text-xs font-medium text-brand-text-label mb-2">API</div>
          <ServiceIndicator name="Backend" status={health.services.api ?? false} />
        </div>
      </div>
    </div>
  )
}
