import { useEffect, useRef, useState } from 'react';
import { fetchEvents } from '../api/client';
import type { ClassificationEvent } from '../api/client';

export function ClassificationLog() {
  const [events, setEvents] = useState<ClassificationEvent[]>([]);
  const [error, setError] = useState<string | null>(null);
  const abortRef = useRef<AbortController | null>(null);
  const scrollRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const tick = async () => {
      abortRef.current?.abort();
      const controller = new AbortController();
      abortRef.current = controller;
      try {
        const data = await fetchEvents(controller.signal);
        setEvents(data);
        setError(null);
        // keep scrolled to top to show latest first
        if (scrollRef.current) scrollRef.current.scrollTop = 0;
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
    <div className="bg-white rounded-lg shadow p-4 h-full flex flex-col">
      <div className="flex items-center justify-between mb-3">
        <h3 className="font-semibold">Classification Log</h3>
        {error && <span className="text-sm text-red-600">{error}</span>}
      </div>
      <div ref={scrollRef} className="flex-1 overflow-auto border rounded">
        <table className="min-w-full text-sm">
          <thead className="bg-gray-50 sticky top-0">
            <tr>
              <th className="text-left p-2">Time</th>
              <th className="text-left p-2">Class</th>
              <th className="text-left p-2">Confidence</th>
            </tr>
          </thead>
          <tbody>
            {events.map(ev => (
              <tr key={ev.id} className="border-t">
                <td className="p-2">{new Date(ev.timestamp * 1000).toLocaleTimeString()}</td>
                <td className="p-2 font-medium">{ev.class}</td>
                <td className="p-2">{Math.round(ev.confidence * 100)}%</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
}

export default ClassificationLog;

