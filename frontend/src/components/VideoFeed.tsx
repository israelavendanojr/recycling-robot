import { useEffect, useRef, useState } from 'react';
import { fetchCounters } from '../api/client';

function sum(values: number[]): number {
  return values.reduce((a, b) => a + b, 0);
}

export function Counters() {
  const [counters, setCounters] = useState<Record<string, number>>({});
  const [error, setError] = useState<string | null>(null);
  const abortRef = useRef<AbortController | null>(null);

  useEffect(() => {
    const tick = async () => {
      // cancel previous in-flight request
      abortRef.current?.abort();
      const controller = new AbortController();
      abortRef.current = controller;

      try {
        const data = await fetchCounters(controller.signal);
        setCounters(data);
        setError(null);
      } catch (e: any) {
        if (e?.name === 'AbortError') return;
        setError(e?.message || 'Failed to load counters');
      }
    };

    const interval = setInterval(tick, 2000);
    tick();
    return () => {
      clearInterval(interval);
      abortRef.current?.abort();
    };
  }, []);

  const total = sum(Object.values(counters));

  return (
    <div className="bg-white rounded-lg shadow p-4">
      <div className="flex items-center justify-between mb-3">
        <h3 className="font-semibold">Counters</h3>
        {error && <span className="text-sm text-red-600">{error}</span>}
      </div>

      <div className="space-y-2">
        {Object.entries(counters).map(([label, count]) => {
          const pct = total ? Math.round((count / total) * 100) : 0;
          return (
            <div key={label}>
              <div className="flex justify-between text-sm">
                <span className="font-medium">{label}</span>
                <span>
                  {count} ({pct}%)
                </span>
              </div>
              <div className="w-full bg-gray-100 h-2 rounded">
                <div
                  className="h-2 rounded bg-green-500"
                  style={{ width: `${pct}%` }}
                />
              </div>
            </div>
          );
        })}

        <div className="pt-2 text-sm text-gray-700">Total: {total}</div>
      </div>
    </div>
  );
}

export default Counters;
