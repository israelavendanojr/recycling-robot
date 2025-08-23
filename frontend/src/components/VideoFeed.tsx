import { useState } from 'react';
import { VIDEO_URL } from '../config';

export function VideoFeed() {
  const [error, setError] = useState(false);
  const [loading, setLoading] = useState(true);

  return (
    <div className="bg-white rounded-xl shadow-lg overflow-hidden">
      <div className="bg-gradient-to-r from-blue-600 to-blue-700 px-6 py-4">
        <h2 className="text-xl font-semibold text-white flex items-center">
          <div className="w-3 h-3 bg-red-400 rounded-full mr-3 animate-pulse"></div>
          Live Camera Feed
        </h2>
      </div>
      
      <div className="p-6">
        <div className="relative bg-gray-900 rounded-lg overflow-hidden aspect-video">
          {error ? (
            <div className="absolute inset-0 flex items-center justify-center bg-gray-100">
              <div className="text-center">
                <div className="w-16 h-16 mx-auto mb-4 bg-gray-300 rounded-full flex items-center justify-center">
                  <svg className="w-8 h-8 text-gray-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 10l4.553-2.276A1 1 0 0121 8.618v6.764a1 1 0 01-1.447.894L15 14M5 18h8a2 2 0 002-2V8a2 2 0 00-2-2H5a2 2 0 00-2 2v8a2 2 0 002 2z" />
                  </svg>
                </div>
                <p className="text-gray-600 font-medium">Stream unavailable</p>
                <p className="text-gray-500 text-sm mt-1">Check camera connection</p>
              </div>
            </div>
          ) : (
            <>
              {loading && (
                <div className="absolute inset-0 flex items-center justify-center bg-gray-900">
                  <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-blue-400"></div>
                </div>
              )}
              <img 
                src={VIDEO_URL}
                alt="Live camera feed"
                className={`w-full h-full object-cover transition-opacity duration-300 ${loading ? 'opacity-0' : 'opacity-100'}`}
                onError={() => setError(true)}
                onLoad={() => {
                  setError(false);
                  setLoading(false);
                }}
              />
            </>
          )}
        </div>
      </div>
    </div>
  );
}

export default VideoFeed;