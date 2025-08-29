import { useState, useEffect } from 'react';
import { fetchCurrentImage } from '../api/client';

export function VideoFeed() {
  const [error, setError] = useState(false);
  const [loading, setLoading] = useState(true);
  const [imageInfo, setImageInfo] = useState<{ image_url: string; timestamp: number; source: string } | null>(null);
  const [debugLog, setDebugLog] = useState<string[]>([]);
  const [showDebug, setShowDebug] = useState(false);

  const addDebugLog = (message: string) => {
    setDebugLog(prev => [...prev.slice(-9), `${new Date().toLocaleTimeString()}: ${message}`]);
  };

      useEffect(() => {
        const fetchImage = async () => {
            try {
                addDebugLog('Fetching current image...');
                const info = await fetchCurrentImage();
                if (info) {
                    setImageInfo(info);
                    setError(false);
                    setLoading(false);
                    addDebugLog(`Successfully loaded image: ${info.image_url}`);
                } else {
                    setError(true);
                    setLoading(false);
                    addDebugLog('No image info received');
                }
            } catch (e) {
                addDebugLog(`Failed to fetch image: ${e}`);
                console.error('[VideoFeed] Failed to fetch image:', e);
                setError(true);
                setLoading(false);
            }
        };

        fetchImage();
        
        // Refresh image every 2 seconds for manual mode (slower polling since images change less frequently)
        const interval = setInterval(fetchImage, 2000);
        return () => clearInterval(interval);
    }, []);

  return (
    <div className="bg-white rounded-xl shadow-sm border border-gray-100 overflow-hidden">
      <div className="bg-gradient-to-r from-blue-600 via-blue-700 to-indigo-700 px-6 py-4">
        <h2 className="text-xl font-bold text-white">Live Camera Feed</h2>
      </div>
      
      <div className="p-6">
        <div className="relative bg-gray-900 rounded-xl overflow-hidden aspect-video border border-gray-200">
          {error ? (
            <div className="absolute inset-0 flex items-center justify-center bg-gradient-to-br from-gray-50 to-gray-100">
              <div className="text-center">
                <div className="w-20 h-20 mx-auto mb-4 bg-gray-200 rounded-full flex items-center justify-center">
                  <svg className="w-10 h-10 text-gray-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 10l4.553-2.276A1 1 0 0121 8.618v6.764a1 1 0 01-1.447.894L15 14M5 18h8a2 2 0 002-2V8a2 2 0 00-2-2H5a2 2 0 00-2 2v8a2 2 0 002 2z" />
                  </svg>
                </div>
                <p className="text-gray-700 font-semibold text-lg">Stream unavailable</p>
                <p className="text-gray-500 text-sm mt-1">Check ROS2 camera connection</p>
              </div>
            </div>
          ) : (
            <>
              {loading && (
                <div className="absolute inset-0 flex items-center justify-center bg-gray-900">
                  <div className="text-center">
                    <div className="animate-spin rounded-full h-16 w-16 border-b-2 border-blue-400 mx-auto mb-4"></div>
                    <p className="text-blue-400 font-medium">Connecting to ROS2 camera...</p>
                  </div>
                </div>
              )}
              
              {imageInfo && !loading && (
                <img 
                  src={imageInfo.image_url}
                  alt="Live camera feed"
                  className="w-full h-full object-cover transition-opacity duration-500 opacity-100"
                  onError={() => {
                    addDebugLog(`Image failed to load: ${imageInfo.image_url}`);
                    console.error('[VideoFeed] Image failed to load:', imageInfo.image_url);
                    setError(true);
                  }}
                  onLoad={() => {
                    addDebugLog(`Image loaded successfully: ${imageInfo.image_url}`);
                  }}
                />
              )}
              
              {/* Stream indicator */}
              {!loading && !error && imageInfo && (
                <div className="absolute bottom-4 right-4 bg-black bg-opacity-50 text-white px-3 py-2 rounded-lg text-sm font-medium">
                  Live • ROS2
                </div>
              )}
            </>
          )}
        </div>
        
        {/* Debug Information - Collapsible */}
        <div className="mt-4 border-t border-gray-100 pt-4">
          <div 
            className="flex items-center justify-between cursor-pointer text-sm text-gray-600 hover:text-gray-800"
            onClick={() => setShowDebug(!showDebug)}
          >
            <span className="font-medium">🔧 Debug Info</span>
            <svg 
              className={`w-4 h-4 transition-transform duration-200 ${showDebug ? 'rotate-180' : ''}`} 
              fill="none" 
              stroke="currentColor" 
              viewBox="0 0 24 24"
            >
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
            </svg>
          </div>
          
          {showDebug && (
            <div className="mt-3 space-y-3">
              {/* Debug Log */}
              <div className="p-3 bg-gray-50 rounded text-xs font-mono max-h-32 overflow-y-auto">
                <div className="font-semibold mb-2 text-gray-700">Debug Log:</div>
                {debugLog.map((log, index) => (
                  <div key={index} className="text-gray-600">{log}</div>
                ))}
              </div>
              
              {/* Image Info Display */}
              {imageInfo && (
                <div className="p-3 bg-blue-50 rounded text-sm">
                  <div><strong>Image URL:</strong> {imageInfo.image_url}</div>
                  <div><strong>Timestamp:</strong> {new Date(imageInfo.timestamp * 1000).toLocaleString()}</div>
                  <div><strong>Source:</strong> {imageInfo.source}</div>
                </div>
              )}
            </div>
          )}
        </div>
      </div>
    </div>
  );
}

export default VideoFeed;