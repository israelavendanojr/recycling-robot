import { useState, useEffect, useRef, useCallback } from 'react';

interface LiveCameraProps {
  onCameraStatusChange?: (status: 'active' | 'inactive' | 'error') => void;
}

export function LiveCamera({ onCameraStatusChange }: LiveCameraProps) {
  const videoRef = useRef<HTMLVideoElement>(null);
  const streamRef = useRef<MediaStream | null>(null);
  
  const [isActive, setIsActive] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [cameraDevices, setCameraDevices] = useState<MediaDeviceInfo[]>([]);
  const [selectedDevice, setSelectedDevice] = useState<string>('');
  const [apiAvailable, setApiAvailable] = useState(false);

  // Check if camera API is available
  useEffect(() => {
    const checkApiAvailability = () => {
      const hasGetUserMedia = !!(navigator.mediaDevices && navigator.mediaDevices.getUserMedia);
      const hasEnumerateDevices = !!(navigator.mediaDevices && navigator.mediaDevices.enumerateDevices);
      
      if (!hasGetUserMedia || !hasEnumerateDevices) {
        setError('Camera API not supported in this browser. Please use a modern browser with camera support.');
        onCameraStatusChange?.('error');
        return false;
      }
      
      setApiAvailable(true);
      return true;
    };

    if (!checkApiAvailability()) {
      return;
    }
  }, [onCameraStatusChange]);

  // Get available camera devices
  const getCameraDevices = useCallback(async () => {
    if (!apiAvailable) return;
    
    try {
      const devices = await navigator.mediaDevices.enumerateDevices();
      const cameras = devices.filter(device => device.kind === 'videoinput');
      setCameraDevices(cameras);
      
      // Auto-select first camera (usually the Logitech C270)
      if (cameras.length > 0 && !selectedDevice) {
        setSelectedDevice(cameras[0].deviceId);
      }
      
      if (cameras.length === 0) {
        setError('No camera devices detected. Please check your camera connection.');
        onCameraStatusChange?.('error');
      }
    } catch (err) {
      console.error('[LiveCamera] Failed to enumerate devices:', err);
      setError('Failed to detect camera devices. Please check permissions.');
      onCameraStatusChange?.('error');
    }
  }, [apiAvailable, selectedDevice, onCameraStatusChange]);

  // Start camera stream
  const startCamera = useCallback(async () => {
    if (!apiAvailable) {
      setError('Camera API not available');
      return;
    }
    
    if (!selectedDevice) {
      setError('No camera device selected');
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      // Stop any existing stream
      if (streamRef.current) {
        streamRef.current.getTracks().forEach(track => track.stop());
        streamRef.current = null;
      }

      // Start new stream with selected device
      const constraints = {
        video: {
          deviceId: { exact: selectedDevice },
          width: { ideal: 1280 },
          height: { ideal: 720 },
          frameRate: { ideal: 30 }
        },
        audio: false
      };

      const stream = await navigator.mediaDevices.getUserMedia(constraints);
      streamRef.current = stream;

      if (videoRef.current) {
        videoRef.current.srcObject = stream;
        videoRef.current.play();
      }

      setIsActive(true);
      onCameraStatusChange?.('active');
      console.log('[LiveCamera] Camera started successfully');
    } catch (err: any) {
      console.error('[LiveCamera] Failed to start camera:', err);
      
      if (err.name === 'NotAllowedError') {
        setError('Camera access denied. Please allow camera permissions and refresh the page.');
      } else if (err.name === 'NotFoundError') {
        setError('Selected camera not found. Please check connection and try again.');
      } else if (err.name === 'NotReadableError') {
        setError('Camera is in use by another application. Please close other camera apps.');
      } else if (err.name === 'OverconstrainedError') {
        setError('Camera does not support requested resolution. Trying with default settings...');
        // Fallback to default constraints
        try {
          const fallbackStream = await navigator.mediaDevices.getUserMedia({ video: true, audio: false });
          streamRef.current = fallbackStream;
          if (videoRef.current) {
            videoRef.current.srcObject = fallbackStream;
            videoRef.current.play();
          }
          setIsActive(true);
          onCameraStatusChange?.('active');
          console.log('[LiveCamera] Camera started with fallback constraints');
        } catch (fallbackErr: any) {
          setError(`Camera failed to start: ${fallbackErr.message || 'Unknown error'}`);
          onCameraStatusChange?.('error');
        }
      } else {
        setError(`Camera error: ${err.message || 'Unknown error'}`);
        onCameraStatusChange?.('error');
      }
    } finally {
      setIsLoading(false);
    }
  }, [apiAvailable, selectedDevice, onCameraStatusChange]);

  // Stop camera stream
  const stopCamera = useCallback(() => {
    if (streamRef.current) {
      streamRef.current.getTracks().forEach(track => track.stop());
      streamRef.current = null;
    }
    
    if (videoRef.current) {
      videoRef.current.srcObject = null;
    }
    
    setIsActive(false);
    onCameraStatusChange?.('inactive');
    console.log('[LiveCamera] Camera stopped');
  }, [onCameraStatusChange]);

  // Initialize camera devices on mount
  useEffect(() => {
    if (apiAvailable) {
      getCameraDevices();
    }
  }, [apiAvailable, getCameraDevices]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (streamRef.current) {
        streamRef.current.getTracks().forEach(track => track.stop());
      }
    };
  }, []);

  // Handle device selection change
  const handleDeviceChange = (deviceId: string) => {
    setSelectedDevice(deviceId);
    if (isActive) {
      stopCamera();
      // Restart with new device after a short delay
      setTimeout(() => startCamera(), 100);
    }
  };

  // If API is not available, show error
  if (!apiAvailable) {
    return (
      <div className="bg-white rounded-xl shadow-sm border border-gray-100 overflow-hidden">
        <div className="bg-gradient-to-r from-red-600 via-red-700 to-red-800 px-6 py-4">
          <div className="flex items-center justify-between">
            <div className="flex items-center">
              <div className="w-3 h-3 bg-red-400 rounded-full mr-3"></div>
              <h2 className="text-xl font-bold text-white">Camera Not Available</h2>
            </div>
          </div>
        </div>
        <div className="p-6">
          <div className="text-center">
            <div className="w-20 h-20 mx-auto mb-4 bg-red-100 rounded-full flex items-center justify-center">
              <svg className="w-10 h-10 text-red-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 9v2m0 4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
              </svg>
            </div>
            <p className="text-red-700 font-semibold text-lg">Browser Not Supported</p>
            <p className="text-red-500 text-sm mt-1">{error}</p>
            <p className="text-gray-600 text-sm mt-3">
              Please use a modern browser like Chrome, Firefox, or Edge with camera permissions enabled.
            </p>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="bg-white rounded-xl shadow-sm border border-gray-100 overflow-hidden">
      <div className="bg-gradient-to-r from-green-600 via-green-700 to-emerald-700 px-6 py-4">
        <div className="flex items-center justify-between">
          <div className="flex items-center">
            <div className={`w-3 h-3 rounded-full mr-3 ${isActive ? 'bg-green-400 animate-pulse' : 'bg-gray-400'}`}></div>
            <h2 className="text-xl font-bold text-white">Live Camera Feed</h2>
          </div>
          <div className="flex items-center space-x-2">
            <div className="flex items-center px-3 py-1 bg-white bg-opacity-20 rounded-full">
              <div className={`w-2 h-2 rounded-full mr-2 ${isActive ? 'bg-green-400 animate-pulse' : 'bg-gray-400'}`}></div>
              <span className="text-sm font-medium text-white">
                {isActive ? 'Live Stream' : 'Offline'}
              </span>
            </div>
          </div>
        </div>
      </div>

      <div className="p-6">
        {/* Camera Controls */}
        <div className="mb-4 flex flex-wrap items-center gap-4">
          {/* Device Selection */}
          <div className="flex-1 min-w-0">
            <label htmlFor="camera-select" className="block text-sm font-medium text-gray-700 mb-1">
              Camera Device
            </label>
            <select
              id="camera-select"
              value={selectedDevice}
              onChange={(e) => handleDeviceChange(e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-green-500 focus:border-green-500"
              disabled={isLoading}
            >
              {cameraDevices.map((device) => (
                <option key={device.deviceId} value={device.deviceId}>
                  {device.label || `Camera ${device.deviceId.slice(0, 8)}...`}
                </option>
              ))}
            </select>
          </div>

          {/* Control Buttons */}
          <div className="flex items-center space-x-2">
            <button
              onClick={startCamera}
              disabled={isLoading || isActive}
              className="px-4 py-2 bg-green-600 text-white rounded-lg hover:bg-green-700 disabled:opacity-50 disabled:cursor-not-allowed transition-colors duration-200 text-sm font-medium"
            >
              {isLoading ? (
                <div className="flex items-center">
                  <div className="animate-spin rounded-full h-4 w-4 border-b-2 border-white mr-2"></div>
                  Starting...
                </div>
              ) : (
                'Start Camera'
              )}
            </button>

            <button
              onClick={stopCamera}
              disabled={!isActive || isLoading}
              className="px-4 py-2 bg-red-600 text-white rounded-lg hover:bg-red-700 disabled:opacity-50 disabled:cursor-not-allowed transition-colors duration-200 text-sm font-medium"
            >
              Stop Camera
            </button>
          </div>
        </div>

        {/* Video Display */}
        <div className="relative bg-gray-900 rounded-xl overflow-hidden aspect-video border border-gray-200">
          {error ? (
            <div className="absolute inset-0 flex items-center justify-center bg-gradient-to-br from-gray-50 to-gray-100">
              <div className="text-center">
                <div className="w-20 h-20 mx-auto mb-4 bg-red-100 rounded-full flex items-center justify-center">
                  <svg className="w-10 h-10 text-red-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-2.5L13.732 4c-.77-.833-1.964-.833-2.732 0L3.732 16.5c-.77.833.192 2.5 1.732 2.5z" />
                  </svg>
                </div>
                <p className="text-red-700 font-semibold text-lg">Camera Error</p>
                <p className="text-red-500 text-sm mt-1">{error}</p>
                <button 
                  onClick={() => {
                    setError(null);
                    startCamera();
                  }}
                  className="mt-3 px-4 py-2 bg-green-600 text-white rounded-lg hover:bg-green-700 transition-colors duration-200 text-sm font-medium"
                >
                  Retry
                </button>
              </div>
            </div>
          ) : (
            <>
              {isLoading && (
                <div className="absolute inset-0 flex items-center justify-center bg-gray-900">
                  <div className="text-center">
                    <div className="animate-spin rounded-full h-16 w-16 border-b-2 border-green-400 mx-auto mb-4"></div>
                    <p className="text-green-400 font-medium">Initializing camera...</p>
                  </div>
                </div>
              )}
              
              <video
                ref={videoRef}
                className={`w-full h-full object-cover transition-opacity duration-500 ${
                  isActive ? 'opacity-100' : 'opacity-0'
                }`}
                autoPlay
                playsInline
                muted
              />
              
              {/* Overlay info when camera is active */}
              {isActive && !isLoading && (
                <div className="absolute bottom-4 right-4 bg-black bg-opacity-50 text-white px-3 py-2 rounded-lg text-sm font-medium">
                  Live • HD
                </div>
              )}
            </>
          )}
        </div>

        {/* Camera Status Info */}
        <div className="mt-4 flex items-center justify-between text-sm text-gray-600">
          <div className="flex items-center space-x-4">
            <div className="flex items-center">
              <div className={`w-2 h-2 rounded-full mr-2 ${isActive ? 'bg-green-500' : 'bg-red-500'}`}></div>
              <span>{isActive ? 'Camera Active' : 'Camera Inactive'}</span>
            </div>
            <div className="flex items-center">
              <div className="w-2 h-2 bg-blue-500 rounded-full mr-2"></div>
              <span>AI Processing Ready</span>
            </div>
          </div>
          <div className="text-gray-500">
            {cameraDevices.length > 0 ? `${cameraDevices.length} camera(s) detected` : 'No cameras detected'}
          </div>
        </div>

        {/* Debug Info */}
        <div className="mt-2 text-xs text-gray-400">
          <p>Selected Device: {selectedDevice ? selectedDevice.slice(0, 20) + '...' : 'None'}</p>
          <p>Stream Status: {isActive ? 'Active' : 'Inactive'}</p>
          <p>Last Updated: {new Date().toLocaleTimeString()}</p>
        </div>
      </div>
    </div>
  );
}

export default LiveCamera;
