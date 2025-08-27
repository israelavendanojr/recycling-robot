import React, { useState, useEffect } from 'react';

interface PipelineState {
  state: 'idle' | 'processing' | 'unknown';
  timestamp: number;
  last_change: number;
  current_item?: {
    class?: string;
    confidence?: number;
  } | null;
}

const PipelineStatus: React.FC = () => {
  const [pipelineState, setPipelineState] = useState<PipelineState | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  const fetchPipelineState = async () => {
    try {
      const response = await fetch('/api/pipeline/state');
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const data = await response.json();
      setPipelineState(data);
      setError(null);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to fetch pipeline state');
      console.error('Error fetching pipeline state:', err);
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    fetchPipelineState();
    
    // Poll for updates every 2 seconds
    const interval = setInterval(fetchPipelineState, 2000);
    
    return () => clearInterval(interval);
  }, []);

  const getStateColor = (state: string) => {
    switch (state) {
      case 'idle':
        return 'bg-green-500';
      case 'processing':
        return 'bg-blue-500';
      default:
        return 'bg-gray-500';
    }
  };

  const getStateText = (state: string) => {
    switch (state) {
      case 'idle':
        return 'Ready';
      case 'processing':
        return 'Processing Item...';
      default:
        return 'Unknown';
    }
  };

  const formatTimestamp = (timestamp: number) => {
    if (!timestamp) return 'N/A';
    return new Date(timestamp * 1000).toLocaleTimeString();
  };

  const getElapsedTime = (timestamp: number) => {
    if (!timestamp) return '';
    const elapsed = Math.floor((Date.now() / 1000) - timestamp);
    if (elapsed < 60) {
      return `${elapsed}s ago`;
    } else if (elapsed < 3600) {
      return `${Math.floor(elapsed / 60)}m ago`;
    } else {
      return `${Math.floor(elapsed / 3600)}h ago`;
    }
  };

  if (loading) {
    return (
      <div className="bg-white rounded-lg shadow-md p-6">
        <div className="flex items-center space-x-3">
          <div className="animate-spin rounded-full h-6 w-6 border-b-2 border-blue-500"></div>
          <span className="text-gray-600">Loading pipeline status...</span>
        </div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="bg-white rounded-lg shadow-md p-6">
        <div className="flex items-center space-x-3">
          <div className="w-3 h-3 bg-red-500 rounded-full"></div>
          <span className="text-red-600">Error: {error}</span>
        </div>
      </div>
    );
  }

  if (!pipelineState) {
    return (
      <div className="bg-white rounded-lg shadow-md p-6">
        <span className="text-gray-600">No pipeline state available</span>
      </div>
    );
  }

  return (
    <div className="bg-white rounded-lg shadow-md p-6">
      <h3 className="text-lg font-semibold text-gray-800 mb-4">Pipeline Status</h3>
      
      {/* Current State */}
      <div className="flex items-center space-x-3 mb-4">
        <div className={`w-4 h-4 rounded-full ${getStateColor(pipelineState.state)}`}></div>
        <span className="text-lg font-medium text-gray-700">
          {getStateText(pipelineState.state)}
        </span>
      </div>

      {/* Current Item Info */}
      {pipelineState.state === 'processing' && pipelineState.current_item && (
        <div className="bg-blue-50 rounded-lg p-4 mb-4">
          <h4 className="font-medium text-blue-800 mb-2">Current Item</h4>
          <div className="space-y-2">
            <div className="flex justify-between">
              <span className="text-blue-700">Material:</span>
              <span className="font-medium text-blue-800 capitalize">
                {pipelineState.current_item.class || 'Unknown'}
              </span>
            </div>
            {pipelineState.current_item.confidence && (
              <div className="flex justify-between">
                <span className="text-blue-700">Confidence:</span>
                <span className="font-medium text-blue-800">
                  {(pipelineState.current_item.confidence * 100).toFixed(1)}%
                </span>
              </div>
            )}
          </div>
        </div>
      )}

      {/* Status Details */}
      <div className="space-y-2 text-sm text-gray-600">
        <div className="flex justify-between">
          <span>Last Updated:</span>
          <span>{formatTimestamp(pipelineState.timestamp)}</span>
        </div>
        <div className="flex justify-between">
          <span>State Changed:</span>
          <span>{getElapsedTime(pipelineState.last_change)}</span>
        </div>
      </div>

      {/* Refresh Button */}
      <button
        onClick={fetchPipelineState}
        className="mt-4 w-full bg-blue-500 hover:bg-blue-600 text-white font-medium py-2 px-4 rounded-md transition-colors duration-200"
      >
        Refresh Status
      </button>
    </div>
  );
};

export default PipelineStatus;
