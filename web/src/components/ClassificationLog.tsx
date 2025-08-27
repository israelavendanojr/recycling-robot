import { useEffect, useRef, useState } from 'react';
import { fetchEvents } from '../api/client';
import type { ClassificationEvent } from '../api/client';

interface ClassificationTableRowProps {
  event: ClassificationEvent;
  isEven: boolean;
}

function ClassificationTableRow({ event, isEven }: ClassificationTableRowProps) {
  const getConfidenceColor = (confidence: number) => {
    if (confidence >= 0.8) return 'text-green-700 bg-green-50 border-green-200';
    if (confidence >= 0.6) return 'text-yellow-700 bg-yellow-50 border-yellow-200';
    return 'text-red-700 bg-red-50 border-red-200';
  };

  const getMaterialColor = (className: string) => {
    const colors: Record<string, string> = {
      cardboard: 'text-amber-700 bg-amber-50 border-amber-200',
      glass: 'text-green-700 bg-green-50 border-green-200',
      metal: 'text-gray-700 bg-gray-50 border-gray-200',
      plastic: 'text-blue-700 bg-blue-50 border-blue-200',
      trash: 'text-red-700 bg-red-50 border-red-200',
    };
    return colors[className.toLowerCase()] || 'text-gray-700 bg-gray-50 border-gray-200';
  };

  return (
    <tr className={`hover:bg-blue-50 transition-all duration-200 ${isEven ? 'bg-gray-50' : 'bg-white'}`}>
      <td className="px-6 py-4 text-sm text-gray-900 font-medium">
        <div className="flex items-center">
          <div className="w-2 h-2 bg-blue-500 rounded-full mr-3"></div>
          {new Date(event.timestamp * 1000).toLocaleTimeString()}
        </div>
      </td>
      <td className="px-6 py-4">
        <span className={`inline-flex px-3 py-1.5 rounded-lg text-sm font-semibold capitalize border ${getMaterialColor(event.class)}`}>
          {event.class}
        </span>
      </td>
      <td className="px-6 py-4">
        <div className="flex items-center space-x-2">
          <span className={`inline-flex px-3 py-1.5 rounded-lg text-sm font-semibold border ${getConfidenceColor(event.confidence)}`}>
            {Math.round(event.confidence * 100)}%
          </span>
          <div className="w-16 bg-gray-200 rounded-full h-2">
            <div 
              className={`h-2 rounded-full transition-all duration-300 ${
                event.confidence >= 0.8 ? 'bg-green-500' : 
                event.confidence >= 0.6 ? 'bg-yellow-500' : 'bg-red-500'
              }`}
              style={{ width: `${event.confidence * 100}%` }}
            ></div>
          </div>
        </div>
      </td>
    </tr>
  );
}

export function ClassificationLog() {
  const [events, setEvents] = useState<ClassificationEvent[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [debugLog, setDebugLog] = useState<string[]>([]);
  const [showDebug, setShowDebug] = useState(false);
  const abortRef = useRef<AbortController | null>(null);

  const addDebugLog = (message: string) => {
    setDebugLog(prev => [...prev.slice(-9), `${new Date().toLocaleTimeString()}: ${message}`]);
  };

  useEffect(() => {
    const tick = async () => {
      abortRef.current?.abort();
      const controller = new AbortController();
      abortRef.current = controller;
      
      try {
        addDebugLog('Fetching events...');
        const data = await fetchEvents(controller.signal);
        setEvents(data);
        setError(null);
        addDebugLog(`Successfully fetched ${data.length} events`);
      } catch (e: any) {
        if (e?.name === 'AbortError') return;
        addDebugLog(`Failed to fetch events: ${e?.message || 'Unknown error'}`);
        console.error('[ClassificationLog] fetchEvents failed', e);
        setError(e?.message || 'Failed to load events');
      }
    };
    
    // Auto-refresh every 1 second for real-time streaming
    const interval = setInterval(tick, 1000);
    tick(); // Initial fetch
    
    return () => {
      clearInterval(interval);
      abortRef.current?.abort();
    };
  }, []);

  return (
    <div className="bg-white rounded-xl shadow-sm border border-gray-100 overflow-hidden">
      <div className="bg-gradient-to-r from-green-600 via-green-700 to-emerald-700 px-6 py-4">
        <h2 className="text-xl font-bold text-white">Classification Logs</h2>
      </div>
      
      <div className="overflow-hidden">
        {error ? (
          <div className="p-6 text-center">
            <div className="w-16 h-16 mx-auto mb-4 bg-red-100 rounded-full flex items-center justify-center">
              <svg className="w-8 h-8 text-red-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4m0 4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
              </svg>
            </div>
            <p className="text-red-700 font-medium">Failed to load logs</p>
            <p className="text-red-500 text-sm mt-1">{error}</p>
          </div>
        ) : (
          <div className="overflow-x-auto">
            <table className="w-full">
              <thead className="bg-gray-50 border-b border-gray-200">
                <tr>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                    Time
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                    Material
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                    Confidence
                  </th>
                </tr>
              </thead>
              <tbody className="divide-y divide-gray-200">
                {events.length === 0 ? (
                  <tr>
                    <td colSpan={3} className="px-6 py-12 text-center text-gray-500">
                      <div className="flex items-center justify-center">
                        <div className="w-8 h-8 bg-gray-200 rounded-full flex items-center justify-center mr-3">
                          <svg className="w-4 h-4 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" />
                          </svg>
                        </div>
                        Waiting for classification events...
                      </div>
                    </td>
                  </tr>
                ) : (
                  events.map((event, index) => (
                    <ClassificationTableRow 
                      key={event.id} 
                      event={event} 
                      isEven={index % 2 === 0}
                    />
                  ))
                )}
              </tbody>
            </table>
          </div>
        )}
        
        {/* Debug Information - Collapsible */}
        <div className="border-t border-gray-200">
          <div 
            className="flex items-center justify-between cursor-pointer p-4 text-sm text-gray-600 hover:text-gray-800 hover:bg-gray-50 transition-colors duration-200"
            onClick={() => setShowDebug(!showDebug)}
          >
            <span className="font-medium">🔧 Debug Info</span>
            <div className="flex items-center space-x-3">
              <span className="text-xs text-gray-500">
                Total: {events.length} | Latest: {events.length > 0 ? events[0]?.class : 'None'}
              </span>
              <svg 
                className={`w-4 h-4 transition-transform duration-200 ${showDebug ? 'rotate-180' : ''}`} 
                fill="none" 
                stroke="currentColor" 
                viewBox="0 0 24 24"
              >
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
              </svg>
            </div>
          </div>
          
          {showDebug && (
            <div className="px-4 pb-4 space-y-3">
              {/* Debug Log */}
              <div className="p-3 bg-gray-50 rounded text-xs font-mono max-h-24 overflow-y-auto">
                <div className="font-semibold mb-2 text-gray-700">Debug Log:</div>
                {debugLog.map((log, index) => (
                  <div key={index} className="text-gray-600">{log}</div>
                ))}
              </div>
              
              {/* Events Info Display */}
              <div className="p-3 bg-blue-50 rounded text-sm">
                <div><strong>Total Events:</strong> {events.length}</div>
                {events.length > 0 && (
                  <>
                    <div><strong>Latest Event:</strong> {new Date(events[0]?.timestamp * 1000).toLocaleString()}</div>
                    <div><strong>Latest Class:</strong> {events[0]?.class}</div>
                  </>
                )}
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  );
}

export default ClassificationLog;