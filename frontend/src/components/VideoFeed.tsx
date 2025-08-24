import { useState, useEffect } from 'react';
import { fetchCurrentImage } from '../api/client';

export function VideoFeed() {
  const [error, setError] = useState(false);
  const [loading, setLoading] = useState(true);
  const [imageInfo, setImageInfo] = useState<{ image_url: string; timestamp: number; source: string } | null>(null);

  useEffect(() => {
    const fetchImage = async () => {
      try {
        console.log('[VideoFeed] Fetching current image...');
        const info = await fetchCurrentImage();
        if (info) {
          setImageInfo(info);
          setError(false);
          setLoading(false);
          console.log('[VideoFeed] Successfully loaded image:', info);
        } else {
          setError(true);
          setLoading(false);
          console.warn('[VideoFeed] No image info received');
        }
      } catch (e) {
        console.error('[VideoFeed] Failed to fetch image:', e);
        setError(true);
        setLoading(false);
      }
    };

    fetchImage();
    
    // Refresh image info every 5 seconds
    const interval = setInterval(fetchImage, 5000);
    return () => clearInterval(interval);
  }, []);

  const handleRetry = () => {
    setLoading(true);
    setError(false);
    window.location.reload();
  };

  return (
    <div className="bg-white rounded-xl shadow-sm border border-gray-100 overflow-hidden">
      <div className="bg-gradient-to-r from-blue-600 via-blue-700 to-indigo-700 px-6 py-4">
        <div className="flex items-center justify-between">
          <div className="flex items-center">
            <div className="w-3 h-3 bg-red-400 rounded-full mr-3 animate-pulse"></div>
            <h2 className="text-xl font-bold text-white">Live Camera Feed</h2>
          </div>
          <div className="flex items-center space-x-2">
            <div className="flex items-center px-3 py-1 bg-white bg-opacity-20 rounded-full">
              <div className="w-2 h-2 bg-green-400 rounded-full mr-2 animate-pulse"></div>
              <span className="text-sm font-medium text-white">
                {imageInfo?.source === 'mock_camera' ? 'Mock Camera' : 'Live Stream'}
              </span>
            </div>
          </div>
        </div>
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
                <p className="text-gray-500 text-sm mt-1">Check camera connection</p>
                <button 
                  onClick={handleRetry}
                  className="mt-3 px-4 py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700 transition-colors duration-200 text-sm font-medium"
                >
                  Retry Connection
                </button>
              </div>
            </div>
          ) : (
            <>
              {loading && (
                <div className="absolute inset-0 flex items-center justify-center bg-gray-900">
                  <div className="text-center">
                    <div className="animate-spin rounded-full h-16 w-16 border-b-2 border-blue-400 mx-auto mb-4"></div>
                    <p className="text-blue-400 font-medium">Initializing camera...</p>
                  </div>
                </div>
              )}
              
              {imageInfo && !loading && (
                <img 
                  src={imageInfo.image_url}
                  alt="Live camera feed"
                  className="w-full h-full object-cover transition-opacity duration-500 opacity-100"
                  onError={() => {
                    console.error('[VideoFeed] Image failed to load:', imageInfo.image_url);
                    setError(true);
                  }}
                />
              )}
              
              {/* Overlay info */}
              {!loading && !error && imageInfo && (
                <div className="absolute bottom-4 right-4 bg-black bg-opacity-50 text-white px-3 py-2 rounded-lg text-sm font-medium">
                  {imageInfo.source === 'mock_camera' ? 'Mock • Test' : 'Live • HD'}
                </div>
              )}
            </>
          )}
        </div>
        
        {/* Camera info */}
        <div className="mt-4 flex items-center justify-between text-sm text-gray-600">
          <div className="flex items-center space-x-4">
            <div className="flex items-center">
              <div className={`w-2 h-2 rounded-full mr-2 ${!error ? 'bg-green-500' : 'bg-red-500'}`}></div>
              <span>{!error ? 'Camera Active' : 'Camera Error'}</span>
            </div>
            <div className="flex items-center">
              <div className="w-2 h-2 bg-blue-500 rounded-full mr-2"></div>
              <span>AI Processing</span>
            </div>
          </div>
          <div className="text-gray-500">
            {imageInfo?.source === 'mock_camera' ? 'Mock camera for testing' : 'Real-time object detection'}
          </div>
        </div>
        
        {/* Debug info */}
        {imageInfo && (
          <div className="mt-2 text-xs text-gray-400">
            <p>Source: {imageInfo.source}</p>
            <p>Last updated: {new Date(imageInfo.timestamp * 1000).toLocaleTimeString()}</p>
          </div>
        )}
      </div>
    </div>
  );
}

export default VideoFeed;