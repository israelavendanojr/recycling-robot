import { useState, useEffect, useRef, useCallback } from 'react'

interface StreamStatus {
  status: 'connecting' | 'connected' | 'error'
  message: string
  lastFrameTime?: Date
  errorMessage?: string
}

interface DebugInfo {
  streamUrl: string
  lastError?: string
  lastFrameTime?: Date
  isHttps: boolean
  isMixedContent: boolean
}

function App() {
  const [streamStatus, setStreamStatus] = useState<StreamStatus>({
    status: 'connecting',
    message: 'Connecting...'
  })
  
  const [debugInfo, setDebugInfo] = useState<DebugInfo>({
    streamUrl: import.meta.env.VITE_STREAM_URL || '/stream/feed.mjpg',
    isHttps: window.location.protocol === 'https:',
    isMixedContent: false
  })
  
  const [showDebug, setShowDebug] = useState(false)
  const [autoReconnect, setAutoReconnect] = useState(true)
  const [fps, setFps] = useState(0)
  
  const imgRef = useRef<HTMLImageElement>(null)
  const reconnectTimeoutRef = useRef<NodeJS.Timeout>()
  const frameCountRef = useRef(0)
  const lastFpsTimeRef = useRef(Date.now())
  const cacheBusterRef = useRef(0)

  // Check for mixed content
  useEffect(() => {
    const isHttps = window.location.protocol === 'https:'
    const isHttpStream = debugInfo.streamUrl.startsWith('http://')
    setDebugInfo(prev => ({
      ...prev,
      isHttps,
      isMixedContent: isHttps && isHttpStream
    }))
  }, [debugInfo.streamUrl])

  // FPS calculation
  const handleFrameLoad = useCallback(() => {
    frameCountRef.current++
    const now = Date.now()
    
    if (now - lastFpsTimeRef.current >= 3000) {
      setFps(Math.round((frameCountRef.current * 1000) / (now - lastFpsTimeRef.current)))
      frameCountRef.current = 0
      lastFpsTimeRef.current = now
    }
    
    setStreamStatus(prev => ({
      status: 'connected',
      message: 'Connected',
      lastFrameTime: new Date()
    }))
    
    setDebugInfo(prev => ({
      ...prev,
      lastFrameTime: new Date()
    }))
  }, [])

  // Error handling with auto-reconnect
  const handleStreamError = useCallback(() => {
    const errorMsg = 'Failed to load video stream'
    setStreamStatus({
      status: 'error',
      message: 'Error',
      errorMessage: errorMsg
    })
    
    setDebugInfo(prev => ({
      ...prev,
      lastError: errorMsg
    }))
    
    // Auto-reconnect after 2 seconds if enabled
    if (autoReconnect) {
      reconnectTimeoutRef.current = setTimeout(() => {
        refreshStream()
      }, 2000)
    }
  }, [autoReconnect])

  // Refresh stream with cache busting
  const refreshStream = useCallback(() => {
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current)
    }
    
    cacheBusterRef.current++
    setStreamStatus({
      status: 'connecting',
      message: 'Connecting...'
    })
    
    if (imgRef.current) {
      imgRef.current.src = `${debugInfo.streamUrl}?t=${cacheBusterRef.current}`
    }
  }, [debugInfo.streamUrl])

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current)
      }
    }
  }, [])

  // Initial stream load
  useEffect(() => {
    refreshStream()
  }, [refreshStream])

  return (
    <div className="min-h-screen bg-gray-100 py-8 px-4">
      <div className="max-w-4xl mx-auto">
        {/* Header */}
        <div className="text-center mb-8">
          <h1 className="text-4xl font-bold text-gray-800 mb-2">
            Recycling Robot — Live Camera
          </h1>
          <p className="text-gray-600">
            Real-time monitoring and control dashboard
          </p>
        </div>

        {/* Main Camera Card */}
        <div className="bg-white rounded-lg shadow-lg p-6 mb-6">
          {/* Status Bar */}
          <div className="flex items-center justify-between mb-4">
            <div className="flex items-center space-x-3">
              <span className={`px-3 py-1 rounded-full text-sm font-medium ${
                streamStatus.status === 'connected' ? 'bg-green-100 text-green-800' :
                streamStatus.status === 'error' ? 'bg-red-100 text-red-800' :
                'bg-yellow-100 text-yellow-800'
              }`}>
                {streamStatus.message}
              </span>
              {fps > 0 && (
                <span className="text-sm text-gray-600">
                  {fps} FPS
                </span>
              )}
            </div>
            
            <div className="flex items-center space-x-2">
              <button
                onClick={refreshStream}
                className="px-4 py-2 bg-blue-600 text-white rounded-md hover:bg-blue-700 transition-colors"
              >
                🔄 Refresh
              </button>
              <label className="flex items-center space-x-2">
                <input
                  type="checkbox"
                  checked={autoReconnect}
                  onChange={(e) => setAutoReconnect(e.target.checked)}
                  className="rounded"
                />
                <span className="text-sm text-gray-700">Auto-reconnect</span>
              </label>
            </div>
          </div>

          {/* Camera Stream */}
          <div className="relative bg-black rounded-lg overflow-hidden">
            {streamStatus.status === 'error' && (
              <div className="absolute inset-0 flex items-center justify-center bg-gray-900 bg-opacity-75 z-10">
                <div className="text-center text-white">
                  <div className="text-6xl mb-4">📹</div>
                  <h3 className="text-xl font-semibold mb-2">Stream Unreachable</h3>
                  <p className="text-gray-300 mb-4">
                    Test URL directly in browser: {debugInfo.streamUrl}
                  </p>
                  <button
                    onClick={refreshStream}
                    className="px-4 py-2 bg-blue-600 text-white rounded-md hover:bg-blue-700"
                  >
                    Try Again
                  </button>
                </div>
              </div>
            )}
            
            <img
              ref={imgRef}
              src={debugInfo.streamUrl}
              alt="Live Camera Feed"
              className="w-full h-auto max-h-96 object-contain"
              onLoad={handleFrameLoad}
              onError={handleStreamError}
              crossOrigin="anonymous"
            />
          </div>

          {/* Mixed Content Warning */}
          {debugInfo.isMixedContent && (
            <div className="mt-4 p-3 bg-yellow-100 border border-yellow-400 rounded-md">
              <div className="flex items-center">
                <span className="text-yellow-800 mr-2">⚠️</span>
                <span className="text-yellow-800 text-sm">
                  <strong>Mixed Content Warning:</strong> App is served over HTTPS but stream is HTTP. 
                  Use the proxy URL <code className="bg-yellow-200 px-1 rounded">/stream/feed.mjpg</code> instead.
                </span>
              </div>
            </div>
          )}
        </div>

        {/* Debug Panel */}
        <div className="bg-white rounded-lg shadow-lg p-6">
          <button
            onClick={() => setShowDebug(!showDebug)}
            className="flex items-center space-x-2 text-gray-700 hover:text-gray-900 mb-4"
          >
            <span className="text-lg">��</span>
            <span className="font-medium">Debug Panel</span>
            <span className={`transform transition-transform ${showDebug ? 'rotate-180' : ''}`}>
              ▼
            </span>
          </button>
          
          {showDebug && (
            <div className="space-y-3 text-sm">
              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                <div>
                  <strong>Stream URL:</strong>
                  <div className="bg-gray-100 p-2 rounded font-mono text-xs break-all">
                    {debugInfo.streamUrl}
                  </div>
                </div>
                
                <div>
                  <strong>Last Frame:</strong>
                  <div className="bg-gray-100 p-2 rounded">
                    {debugInfo.lastFrameTime ? 
                      debugInfo.lastFrameTime.toLocaleTimeString() : 
                      'Never'
                    }
                  </div>
                </div>
              </div>
              
              {debugInfo.lastError && (
                <div>
                  <strong>Last Error:</strong>
                  <div className="bg-red-100 p-2 rounded text-red-800">
                    {debugInfo.lastError}
                  </div>
                </div>
              )}
              
              <div className="grid grid-cols-1 md:grid-cols-3 gap-4 text-xs">
                <div>
                  <strong>Protocol:</strong> {window.location.protocol}
                </div>
                <div>
                  <strong>HTTPS:</strong> {debugInfo.isHttps ? 'Yes' : 'No'}
                </div>
                <div>
                  <strong>Mixed Content:</strong> {debugInfo.isMixedContent ? 'Yes' : 'No'}
                </div>
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  )
}

export default App