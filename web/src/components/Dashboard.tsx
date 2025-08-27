import VideoFeed from './VideoFeed';
import ClassificationLog from './ClassificationLog';
import { useState, useEffect } from 'react';
import { fetchHealth, fetchCurrentImage, fetchEvents } from '../api/client';

export function Dashboard() {
  const [debugInfo, setDebugInfo] = useState<any>({});
  const [lastUpdate, setLastUpdate] = useState<Date>(new Date());
  const [showDebug, setShowDebug] = useState(false);

  console.log('📊 Dashboard component rendered at:', new Date().toISOString());

  useEffect(() => {
    const updateDebugInfo = async () => {
      try {
        console.log('🔄 Updating debug info...');
        const health = await fetchHealth();
        const imageInfo = await fetchCurrentImage();
        const events = await fetchEvents();
        
        setDebugInfo({
          health,
          imageInfo,
          eventsCount: events.length,
          timestamp: new Date().toISOString()
        });
        setLastUpdate(new Date());
        console.log('✅ Debug info updated successfully');
      } catch (error: any) {
        console.error('❌ Debug info fetch error:', error);
        setDebugInfo({ error: error.message });
      }
    };

    updateDebugInfo();
    const interval = setInterval(updateDebugInfo, 5000);
    return () => clearInterval(interval);
  }, []);

  return (
    <div className="min-h-screen bg-gray-50 p-6">
      <div className="max-w-7xl mx-auto">
        
        {/* Two Component Layout */}
        <div className="grid grid-cols-1 xl:grid-cols-2 gap-8 mb-8">
          {/* Left Column - Camera Feed */}
          <VideoFeed />
          
          {/* Right Column - Classification Logs */}
          <ClassificationLog />
        </div>
        
        {/* Debug Information - Collapsible at bottom */}
        <div className="bg-white rounded-xl shadow-sm border border-gray-100 overflow-hidden">
          <div 
            className="bg-gradient-to-r from-gray-600 via-gray-700 to-gray-800 px-6 py-3 cursor-pointer hover:from-gray-700 hover:via-gray-800 hover:to-gray-900 transition-all duration-200"
            onClick={() => setShowDebug(!showDebug)}
          >
            <div className="flex items-center justify-between">
              <h3 className="text-lg font-semibold text-white">🔧 Debug Information</h3>
              <div className="flex items-center space-x-2">
                <span className="text-xs text-gray-300">
                  Last update: {lastUpdate.toLocaleTimeString()}
                </span>
                <svg 
                  className={`w-5 h-5 text-white transition-transform duration-200 ${showDebug ? 'rotate-180' : ''}`} 
                  fill="none" 
                  stroke="currentColor" 
                  viewBox="0 0 24 24"
                >
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
                </svg>
              </div>
            </div>
          </div>
          
          {showDebug && (
            <div className="p-6 border-t border-gray-100">
              <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4 text-sm mb-4">
                <div className="bg-gray-50 p-3 rounded-lg">
                  <strong>Backend Health:</strong> 
                  <span className={`ml-2 px-2 py-1 rounded text-xs ${
                    debugInfo.health?.status === 'ok' ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'
                  }`}>
                    {debugInfo.health?.status || 'unknown'}
                  </span>
                </div>
                <div className="bg-gray-50 p-3 rounded-lg">
                  <strong>Image Available:</strong> 
                  <span className={`ml-2 px-2 py-1 rounded text-xs ${
                    debugInfo.imageInfo ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'
                  }`}>
                    {debugInfo.imageInfo ? 'Yes' : 'No'}
                  </span>
                </div>
                <div className="bg-gray-50 p-3 rounded-lg">
                  <strong>Events Count:</strong> 
                  <span className="ml-2 px-2 py-1 rounded text-xs bg-blue-100 text-blue-800">
                    {debugInfo.eventsCount || 0}
                  </span>
                </div>
                <div className="bg-gray-50 p-3 rounded-lg">
                  <strong>Update Interval:</strong> 
                  <span className="ml-2 px-2 py-1 rounded text-xs bg-purple-100 text-purple-800">
                    5s
                  </span>
                </div>
              </div>
              
              {debugInfo.error && (
                <div className="mb-4 p-3 bg-red-50 border border-red-200 rounded text-red-700 text-sm">
                  <strong>Error:</strong> {debugInfo.error}
                </div>
              )}
              
              <details className="text-sm">
                <summary className="cursor-pointer text-gray-600 hover:text-gray-800 font-medium">
                  Raw Debug Data
                </summary>
                <pre className="mt-2 p-3 bg-gray-50 rounded text-xs overflow-auto max-h-40 border">
                  {JSON.stringify(debugInfo, null, 2)}
                </pre>
              </details>
            </div>
          )}
        </div>
      </div>
    </div>
  );
}

export default Dashboard;
