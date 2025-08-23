import { useEffect, useRef, useState } from 'react';
import { fetchCounters } from '../api/client';

interface MetricCardProps {
  label: string;
  value: number;
  total: number;
  color: string;
}

function MetricCard({ label, value, total, color }: MetricCardProps) {
  const percentage = total > 0 ? Math.round((value / total) * 100) : 0;
  const circumference = 2 * Math.PI * 45;
  const strokeDashoffset = circumference - (percentage / 100) * circumference;

  return (
    <div className="bg-white rounded-lg shadow-md p-6 hover:shadow-lg transition-shadow duration-200">
      <div className="flex items-center justify-between">
        <div className="flex-1">
          <h3 className="text-sm font-medium text-gray-500 uppercase tracking-wider">{label}</h3>
          <div className="mt-2">
            <p className="text-3xl font-bold text-gray-900">{value}</p>
            <p className="text-sm text-gray-600 mt-1">{percentage}% of total</p>
          </div>
        </div>
        
        <div className="relative w-20 h-20">
          <svg className="w-20 h-20 transform -rotate-90" viewBox="0 0 100 100">
            {/* Background circle */}
            <circle
              cx="50"
              cy="50"
              r="45"
              stroke="currentColor"
              strokeWidth="6"
              fill="transparent"
              className="text-gray-200"
            />
            {/* Progress circle */}
            <circle
              cx="50"
              cy="50"
              r="45"
              stroke="currentColor"
              strokeWidth="6"
              fill="transparent"
              strokeDasharray={circumference}
              strokeDashoffset={strokeDashoffset}
              strokeLinecap="round"
              className={`${color} transition-all duration-500 ease-in-out`}
            />
          </svg>
          <div className="absolute inset-0 flex items-center justify-center">
            <span className="text-lg font-semibold text-gray-700">{percentage}%</span>
          </div>
        </div>
      </div>
    </div>
  );
}

const MATERIAL_COLORS: Record<string, string> = {
  cardboard: 'text-amber-500',
  glass: 'text-green-500',
  metal: 'text-gray-500',
  plastic: 'text-blue-500',
  trash: 'text-red-500',
};

function sum(values: number[]): number {
  return values.reduce((a, b) => a + b, 0);
}

export function Counters() {
  const [counters, setCounters] = useState<Record<string, number>>({});
  const [error, setError] = useState<string | null>(null);
  const abortRef = useRef<AbortController | null>(null);

  useEffect(() => {
    const tick = async () => {
      abortRef.current?.abort();
      const controller = new AbortController();
      abortRef.current = controller;
      try {
        const data = await fetchCounters(controller.signal);
        setCounters(data);
        setError(null);
      } catch (e: any) {
        if (e?.name === 'AbortError') return;
        console.error('[Counters] fetchCounters failed', e);
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
    <div className="space-y-6">
      <div className="bg-white rounded-xl shadow-lg p-6">
        <div className="flex items-center justify-between mb-6">
          <h2 className="text-xl font-semibold text-gray-900">Classification Metrics</h2>
          {error ? (
            <span className="text-sm text-red-600 bg-red-50 px-3 py-1 rounded-full">{error}</span>
          ) : (
            <span className="text-sm text-green-600 bg-green-50 px-3 py-1 rounded-full">Live</span>
          )}
        </div>

        {Object.keys(counters).length === 0 ? (
          <div className="text-center py-12">
            <div className="w-16 h-16 mx-auto mb-4 bg-gray-100 rounded-full flex items-center justify-center">
              <svg className="w-8 h-8 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 19v-6a2 2 0 00-2-2H5a2 2 0 00-2 2v6a2 2 0 002 2h2a2 2 0 002-2zm0 0V9a2 2 0 012-2h2a2 2 0 012 2v10m-6 0a2 2 0 002 2h2a2 2 0 002-2m0 0V5a2 2 0 012-2h2a2 2 0 012 2v14a2 2 0 01-2 2h-2a2 2 0 01-2-2z" />
              </svg>
            </div>
            <p className="text-gray-500 font-medium">No classifications yet</p>
            <p className="text-gray-400 text-sm mt-1">Start the classifier to see metrics</p>
          </div>
        ) : (
          <div className="grid grid-cols-1 sm:grid-cols-2 gap-4">
            {Object.entries(counters).map(([label, count]) => (
              <MetricCard
                key={label}
                label={label}
                value={count}
                total={total}
                color={MATERIAL_COLORS[label.toLowerCase()] || 'text-gray-500'}
              />
            ))}
          </div>
        )}

        {total > 0 && (
          <div className="mt-6 pt-6 border-t border-gray-200">
            <div className="bg-gray-50 rounded-lg p-4">
              <div className="flex justify-between items-center">
                <span className="text-lg font-medium text-gray-700">Total Classifications</span>
                <span className="text-2xl font-bold text-gray-900">{total}</span>
              </div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}

export default Counters;