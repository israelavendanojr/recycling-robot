import { useState } from 'react';

export function LiveFeed() {
  const [error, setError] = useState(false);

  return (
    <div className="bg-white rounded-lg shadow p-6">
      <h2 className="text-xl font-semibold mb-4">Live Camera Feed</h2>
      
      {error ? (
        <div className="bg-gray-200 rounded-lg h-64 flex items-center justify-center">
          <p className="text-gray-600">Stream unavailable</p>
        </div>
      ) : (
        <img 
          src="/api/stream" 
          alt="Live camera feed"
          className="w-full h-auto rounded-lg"
          onError={() => setError(true)}
          onLoad={() => setError(false)}
        />
      )}
    </div>
  );
}