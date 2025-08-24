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
  const [lastUpdate, setLastUpdate] = useState<Date>(new Date());
  const abortRef = useRef<AbortController | null>(null);

  useEffect(() => {
    const tick = async () => {
      abortRef.current?.abort();
      const controller = new AbortController();
      abortRef.current = controller;
      
      try {
        console.log('[ClassificationLog] Fetching events...');
        const data = await fetchEvents(controller.signal);
        
        if (data.length !== events.length) {
          console.log(`[ClassificationLog] Events updated: ${events.length} → ${data.length}`);
        }
        
        setEvents(data);
        setError(null);
        setLastUpdate(new Date());
        
        if (data.length > 0) {
          console.log(`[ClassificationLog] Latest event: ${data[0].class} (${Math.round(data[0].confidence * 100)}%) at ${new Date(data[0].timestamp * 1000).toLocaleTimeString()}`);
        }
        
      } catch (e: any) {
        if (e?.name === 'AbortError') return;
        console.error('[ClassificationLog] fetchEvents failed', e);
        setError(e?.message || 'Failed to load events');
      }
    };
    
    const interval = setInterval(tick, 2000);
    tick(); // Initial fetch
    
    console.log('[ClassificationLog] Component mounted, starting event polling');
    
    return () => {
      console.log('[ClassificationLog] Component unmounting, cleaning up');
      clearInterval(interval);
      abortRef.current?.abort();
    };
  }, [events.length]);

  return (
    <div className="bg-white rounded-xl shadow-sm border border-gray-100 overflow-hidden">
      <div className="bg-gradient-to-r from-gray-800 via-gray-900 to-black px-6 py-4">
        <div className="flex items-center justify-between">
          <div className="flex items-center">
            <div className="w-10 h-10 bg-gray-700 rounded-lg flex items-center justify-center mr-4">
              <svg className="w-6 h-6 text-gray-300" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" />
              </svg>
            </div>
            <div>
              <h2 className="text-xl font-bold text-white">Classification Log</h2>
              <p className="text-gray-300 text-sm mt-1">Real-time AI detection results</p>
            </div>
          </div>
          {error ? (
            <span className="inline-flex items-center px-4 py-2 rounded-full text-sm font-semibold bg-red-900 bg-opacity-30 text-red-300 border border-red-700">
              <div className="w-2 h-2 bg-red-400 rounded-full mr-2"></div>
              {error}
            </span>
          ) : (
            <span className="inline-flex items-center px-4 py-2 rounded-full text-sm font-semibold bg-green-900 bg-opacity-30 text-green-300 border border-green-700">
              <div className="w-2 h-2 bg-green-400 rounded-full mr-2 animate-pulse"></div>
              Live Updates
            </span>
          )}
        </div>
      </div>

      <div className="max-h-96 overflow-auto">
        {events.length === 0 ? (
          <div className="text-center py-16">
            <div className="w-20 h-20 mx-auto mb-4 bg-gray-100 rounded-full flex items-center justify-center">
              <svg className="w-10 h-10 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" />
              </svg>
            </div>
            <p className="text-gray-600 font-semibold text-lg">No events logged yet</p>
            <p className="text-gray-500 text-sm mt-1">Classifications will appear here in real-time</p>
            <div className="mt-4 flex items-center justify-center space-x-2 text-sm text-gray-400">
              <div className="w-2 h-2 bg-blue-500 rounded-full animate-pulse"></div>
              <span>Waiting for AI detection...</span>
            </div>
          </div>
        ) : (
          <table className="min-w-full">
            <thead className="bg-gray-50 sticky top-0 border-b border-gray-200">
              <tr>
                <th className="px-6 py-4 text-left text-sm font-bold text-gray-900 uppercase tracking-wider">
                  <div className="flex items-center">
                    <svg className="w-4 h-4 text-gray-500 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z" />
                    </svg>
                    Time
                  </div>
                </th>
                <th className="px-6 py-4 text-left text-sm font-bold text-gray-900 uppercase tracking-wider">
                  <div className="flex items-center">
                    <svg className="w-4 h-4 text-gray-500 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M7 7h.01M7 3h5c.512 0 1.024.195 1.414.586l7 7a2 2 0 010 2.828l-7 7a2 2 0 01-2.828 0l-7-7A1.994 1.994 0 013 12V7a4 4 0 014-4z" />
                    </svg>
                    Material
                  </div>
                </th>
                <th className="px-6 py-4 text-left text-sm font-bold text-gray-900 uppercase tracking-wider">
                  <div className="flex items-center">
                    <svg className="w-4 h-4 text-gray-500 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" />
                    </svg>
                    Confidence
                  </div>
                </th>
              </tr>
            </thead>
            <tbody className="divide-y divide-gray-100">
              {events.map((event, index) => (
                <ClassificationTableRow
                  key={event.id}
                  event={event}
                  isEven={index % 2 === 0}
                />
              ))}
            </tbody>
          </table>
        )}
      </div>
      
      {events.length > 0 && (
        <div className="px-6 py-4 bg-gray-50 border-t border-gray-200">
          <div className="flex items-center justify-between text-sm text-gray-600">
            <span className="font-medium">Total Events: {events.length}</span>
            <span>Last updated: {lastUpdate.toLocaleTimeString()}</span>
          </div>
        </div>
      )}
    </div>
  );
}

export default ClassificationLog;