import { useEffect, useRef, useState } from 'react';
import { fetchEvents } from '../api/client';
import type { ClassificationEvent } from '../api/client';

interface ClassificationTableRowProps {
  event: ClassificationEvent;
  isEven: boolean;
}

function ClassificationTableRow({ event, isEven }: ClassificationTableRowProps) {
  const getConfidenceColor = (confidence: number) => {
    if (confidence >= 0.8) return 'text-green-600 bg-green-50';
    if (confidence >= 0.6) return 'text-yellow-600 bg-yellow-50';
    return 'text-red-600 bg-red-50';
  };

  const getMaterialColor = (className: string) => {
    const colors: Record<string, string> = {
      cardboard: 'text-amber-700 bg-amber-50',
      glass: 'text-green-700 bg-green-50',
      metal: 'text-gray-700 bg-gray-50',
      plastic: 'text-blue-700 bg-blue-50',
      trash: 'text-red-700 bg-red-50',
    };
    return colors[className.toLowerCase()] || 'text-gray-700 bg-gray-50';
  };

  return (
    <tr className={`hover:bg-blue-50 transition-colors duration-150 ${isEven ? 'bg-gray-50' : 'bg-white'}`}>
      <td className="px-4 py-3 text-sm text-gray-900 font-medium">
        {new Date(event.timestamp * 1000).toLocaleTimeString()}
      </td>
      <td className="px-4 py-3">
        <span className={`inline-flex px-3 py-1 rounded-full text-sm font-medium capitalize ${getMaterialColor(event.class)}`}>
          {event.class}
        </span>
      </td>
      <td className="px-4 py-3">
        <span className={`inline-flex px-3 py-1 rounded-full text-sm font-medium ${getConfidenceColor(event.confidence)}`}>
          {Math.round(event.confidence * 100)}%
        </span>
      </td>
    </tr>
  );
}

export function ClassificationLog() {
  const [events, setEvents] = useState<ClassificationEvent[]>([]);
  const [error, setError] = useState<string | null>(null);
  const abortRef = useRef<AbortController | null>(null);

  useEffect(() => {
    const tick = async () => {
      abortRef.current?.abort();
      const controller = new AbortController();
      abortRef.current = controller;
      try {
        const data = await fetchEvents(controller.signal);
        setEvents(data);
        setError(null);
      } catch (e: any) {
        if (e?.name === 'AbortError') return;
        console.error('[ClassificationLog] fetchEvents failed', e);
        setError(e?.message || 'Failed to load events');
      }
    };
    const interval = setInterval(tick, 2000);
    tick();
    return () => {
      clearInterval(interval);
      abortRef.current?.abort();
    };
  }, []);

  return (
    <div className="bg-white rounded-xl shadow-lg overflow-hidden">
      <div className="bg-gradient-to-r from-gray-800 to-gray-900 px-6 py-4">
        <div className="flex items-center justify-between">
          <h2 className="text-xl font-semibold text-white">Classification Log</h2>
          {error ? (
            <span className="text-sm text-red-300 bg-red-900 bg-opacity-30 px-3 py-1 rounded-full">
              {error}
            </span>
          ) : (
            <span className="text-sm text-green-300 bg-green-900 bg-opacity-30 px-3 py-1 rounded-full">
              Live Updates
            </span>
          )}
        </div>
      </div>

      <div className="max-h-96 overflow-auto">
        {events.length === 0 ? (
          <div className="text-center py-12">
            <div className="w-16 h-16 mx-auto mb-4 bg-gray-100 rounded-full flex items-center justify-center">
              <svg className="w-8 h-8 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" />
              </svg>
            </div>
            <p className="text-gray-500 font-medium">No events logged yet</p>
            <p className="text-gray-400 text-sm mt-1">Classifications will appear here in real-time</p>
          </div>
        ) : (
          <table className="min-w-full">
            <thead className="bg-gray-100 sticky top-0">
              <tr>
                <th className="px-4 py-3 text-left text-sm font-semibold text-gray-900 uppercase tracking-wider">
                  Time
                </th>
                <th className="px-4 py-3 text-left text-sm font-semibold text-gray-900 uppercase tracking-wider">
                  Material
                </th>
                <th className="px-4 py-3 text-left text-sm font-semibold text-gray-900 uppercase tracking-wider">
                  Confidence
                </th>
              </tr>
            </thead>
            <tbody>
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
    </div>
  );
}

export default ClassificationLog;