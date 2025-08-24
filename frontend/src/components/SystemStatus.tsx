import { useState, useEffect } from 'react';
import { fetchHealth } from '../api/client';

interface SystemStatusProps {
  cameraStatus?: 'active' | 'inactive' | 'error';
}

interface HealthStatus {
  ok: boolean;
  services: {
    camera: boolean;
    db: boolean;
    ros2: boolean;
  };
  system?: {
    cpu_percent: number;
    memory_percent: number;
    disk_usage: number;
    uptime: number;
  };
  timestamp: number;
  classifier_running: boolean;
}

export function SystemStatus({ cameraStatus }: SystemStatusProps) {
  const [healthData, setHealthData] = useState<HealthStatus | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [lastUpdate, setLastUpdate] = useState<Date>(new Date());

  const fetchHealthData = async () => {
    try {
      setLoading(true);
      const data = await fetchHealth();
      setHealthData(data);
      setError(null);
      setLastUpdate(new Date());
    } catch (err: any) {
      console.error('[SystemStatus] Failed to fetch health:', err);
      setError(err?.message || 'Health check failed');
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    fetchHealthData();
    const interval = setInterval(fetchHealthData, 10000); // Update every 10 seconds
    return () => clearInterval(interval);
  }, []);

  const getStatusColor = (status: boolean) => {
    return status ? 'bg-green-500' : 'bg-red-500';
  };



  const getComponentStatus = (component: string) => {
    if (!healthData) return false;
    
    // For camera, prioritize live camera status if available
    if (component === 'camera' && cameraStatus) {
      return cameraStatus === 'active';
    }
    
    return healthData.services[component as keyof typeof healthData.services] || false;
  };

  const formatUptime = (seconds: number) => {
    const days = Math.floor(seconds / 86400);
    const hours = Math.floor((seconds % 86400) / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    
    if (days > 0) return `${days}d ${hours}h`;
    if (hours > 0) return `${hours}h ${minutes}m`;
    return `${minutes}m`;
  };

  if (loading && !healthData) {
    return (
      <div className="bg-white rounded-xl shadow-sm border border-gray-100 p-6">
        <div className="flex items-center justify-center py-8">
          <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-600"></div>
          <span className="ml-3 text-gray-600">Loading system status...</span>
        </div>
      </div>
    );
  }

  return (
    <div className="bg-white rounded-xl shadow-sm border border-gray-100 p-6">
      <div className="flex items-center justify-between mb-6">
        <div>
          <h2 className="text-xl font-bold text-gray-900">System Status</h2>
          <p className="text-gray-600 text-sm mt-1">Real-time component health monitoring</p>
        </div>
        <div className={`flex items-center px-4 py-2 rounded-full text-sm font-semibold transition-all duration-200 ${
          healthData?.ok ? 'bg-green-100 text-green-800 border border-green-200' : 'bg-red-100 text-red-800 border border-red-200'
        }`}>
          <div className={`w-2 h-2 rounded-full mr-2 ${healthData?.ok ? 'bg-green-500' : 'bg-red-500'}`}></div>
          {healthData?.ok ? 'All Systems Operational' : 'System Issues Detected'}
        </div>
      </div>

      {error && (
        <div className="mb-6 p-4 bg-red-50 border border-red-200 rounded-xl">
          <div className="flex items-center">
            <svg className="w-5 h-5 text-red-500 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4m0 4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
            </svg>
            <p className="text-sm text-red-700 font-medium">{error}</p>
          </div>
        </div>
      )}

      {/* Component Status Grid */}
      <div className="grid grid-cols-1 md:grid-cols-3 gap-4 mb-6">
        {/* Camera Status */}
        <div className="text-center group">
          <div className={`w-16 h-16 mx-auto mb-3 rounded-xl flex items-center justify-center transition-all duration-200 ${
            getComponentStatus('camera') ? 'bg-green-100 group-hover:bg-green-200' : 'bg-red-100 group-hover:bg-red-200'
          }`}>
            {getComponentStatus('camera') ? (
              <svg className="w-8 h-8 text-green-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 10l4.553-2.276A1 1 0 0121 8.618v6.764a1 1 0 01-1.447.894L15 14M5 18h8a2 2 0 002-2V8a2 2 0 00-2-2H5a2 2 0 00-2 2v8a2 2 0 002 2z" />
              </svg>
            ) : (
              <svg className="w-8 h-8 text-red-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 10l4.553-2.276A1 1 0 0121 8.618v6.764a1 1 0 01-1.447.894L15 14M5 18h8a2 2 0 002-2V8a2 2 0 00-2-2H5a2 2 0 00-2 2v8a2 2 0 002 2z" />
              </svg>
            )}
          </div>
          <p className="text-sm font-medium text-gray-600 mb-1">Camera</p>
          <div className={`w-3 h-3 mx-auto rounded-full ${getStatusColor(getComponentStatus('camera'))}`}></div>
          <p className="text-xs text-gray-500 mt-1">
            {cameraStatus === 'active' ? 'Live Feed Active' : 
             cameraStatus === 'error' ? 'Hardware Error' : 'Offline'}
          </p>
        </div>

        {/* Database Status */}
        <div className="text-center group">
          <div className={`w-16 h-16 mx-auto mb-3 rounded-xl flex items-center justify-center transition-all duration-200 ${
            getComponentStatus('db') ? 'bg-green-100 group-hover:bg-green-200' : 'bg-red-100 group-hover:bg-red-200'
          }`}>
            {getComponentStatus('db') ? (
              <svg className="w-8 h-8 text-green-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 7v10c0 2.21 3.582 4 8 4s8-1.79 8-4V7M4 7c0 2.21 3.582 4 8 4s8-1.79 8-4M4 7c0-2.21 3.582-4 8-4s8 1.79 8 4" />
              </svg>
            ) : (
              <svg className="w-8 h-8 text-red-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 7v10c0 2.21 3.582 4 8 4s8-1.79 8-4V7M4 7c0 2.21 3.582 4 8 4s8-1.79 8-4M4 7c0-2.21 3.582-4 8-4s8 1.79 8 4" />
              </svg>
            )}
          </div>
          <p className="text-sm font-medium text-gray-600 mb-1">Database</p>
          <div className={`w-3 h-3 mx-auto rounded-full ${getStatusColor(getComponentStatus('db'))}`}></div>
          <p className="text-xs text-gray-500 mt-1">
            {getComponentStatus('db') ? 'Connected & Healthy' : 'Connection Failed'}
          </p>
        </div>

        {/* ROS2 Status */}
        <div className="text-center group">
          <div className={`w-16 h-16 mx-auto mb-3 rounded-xl flex items-center justify-center transition-all duration-200 ${
            getComponentStatus('ros2') ? 'bg-green-100 group-hover:bg-green-200' : 'bg-red-100 group-hover:bg-red-200'
          }`}>
            {getComponentStatus('ros2') ? (
              <svg className="w-8 h-8 text-green-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
              </svg>
            ) : (
              <svg className="w-8 h-8 text-red-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
              </svg>
            )}
          </div>
          <p className="text-sm font-medium text-gray-600 mb-1">ROS2</p>
          <div className={`w-3 h-3 mx-auto rounded-full ${getStatusColor(getComponentStatus('ros2'))}`}></div>
          <p className="text-xs text-gray-500 mt-1">
            {getComponentStatus('ros2') ? 'Nodes Running' : 'No Active Nodes'}
          </p>
        </div>
      </div>

      {/* System Metrics */}
      {healthData?.system && (
        <div className="pt-6 border-t border-gray-200">
          <h3 className="text-sm font-semibold text-gray-700 mb-4 uppercase tracking-wider">System Metrics</h3>
          <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
            <div className="text-center">
              <div className="text-lg font-bold text-blue-600">{healthData.system.cpu_percent.toFixed(1)}%</div>
              <div className="text-xs text-gray-500">CPU Usage</div>
              <div className="w-full bg-gray-200 rounded-full h-2 mt-2">
                <div className="bg-blue-600 h-2 rounded-full" style={{ width: `${healthData.system.cpu_percent}%` }}></div>
              </div>
            </div>
            
            <div className="text-center">
              <div className="text-lg font-bold text-green-600">{healthData.system.memory_percent.toFixed(1)}%</div>
              <div className="text-xs text-gray-500">Memory</div>
              <div className="w-full bg-gray-200 rounded-full h-2 mt-2">
                <div className="bg-green-600 h-2 rounded-full" style={{ width: `${healthData.system.memory_percent}%` }}></div>
              </div>
            </div>
            
            <div className="text-center">
              <div className="text-lg font-bold text-purple-600">{healthData.system.disk_usage.toFixed(1)}%</div>
              <div className="text-xs text-gray-500">Disk</div>
              <div className="w-full bg-gray-200 rounded-full h-2 mt-2">
                <div className="bg-purple-600 h-2 rounded-full" style={{ width: `${healthData.system.disk_usage}%` }}></div>
              </div>
            </div>
            
            <div className="text-center">
              <div className="text-lg font-bold text-indigo-600">{formatUptime(healthData.system.uptime)}</div>
              <div className="text-xs text-gray-500">Uptime</div>
            </div>
          </div>
        </div>
      )}

      {/* Classifier Status */}
      <div className="pt-6 border-t border-gray-200">
        <div className="flex items-center justify-between">
          <div className="flex items-center">
            <div className={`w-3 h-3 rounded-full mr-2 ${healthData?.classifier_running ? 'bg-green-500' : 'bg-red-500'}`}></div>
            <span className="text-sm font-medium text-gray-700">AI Classifier</span>
          </div>
          <span className={`text-sm px-2 py-1 rounded-full ${
            healthData?.classifier_running 
              ? 'bg-green-100 text-green-800' 
              : 'bg-red-100 text-red-800'
          }`}>
            {healthData?.classifier_running ? 'Running' : 'Stopped'}
          </span>
        </div>
      </div>

      {/* Last Update */}
      <div className="mt-4 text-xs text-gray-400 text-center">
        Last updated: {lastUpdate.toLocaleTimeString()}
        <button 
          onClick={fetchHealthData}
          className="ml-2 text-blue-500 hover:text-blue-700 underline"
        >
          Refresh
        </button>
      </div>
    </div>
  );
}

export default SystemStatus;
