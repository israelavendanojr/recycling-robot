import { useEffect, useRef, useState } from 'react';
import { VIDEO_URL } from '../config';

export function VideoFeed() {
  const imgRef = useRef<HTMLImageElement>(null);
  const [url] = useState<string>(VIDEO_URL);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (!imgRef.current) return;
    const img = imgRef.current;
    let cache = Date.now();
    img.src = `${url}?t=${cache}`;
    const onError = () => setError('Waiting for camera...');
    const onLoad = () => setError(null);
    img.addEventListener('error', onError);
    img.addEventListener('load', onLoad);
    return () => {
      img.removeEventListener('error', onError);
      img.removeEventListener('load', onLoad);
    };
  }, [url]);

  return (
    <div className="bg-black rounded-lg overflow-hidden flex items-center justify-center min-h-64">
      {error ? (
        <div className="text-center p-8 text-white">
          <div className="text-5xl mb-3">📷</div>
          <div className="text-lg mb-2">{error}</div>
          <div className="text-sm opacity-70">Mock feed until camera is available</div>
        </div>
      ) : (
        <img ref={imgRef} alt="Video" className="w-full h-auto object-contain max-h-96" />
      )}
    </div>
  );
}

export default VideoFeed;

