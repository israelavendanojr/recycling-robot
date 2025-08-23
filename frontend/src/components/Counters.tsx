import { useEffect, useRef, useState } from 'react';
import { fetchCounters } from '../api/client';

interface MetricCardProps {
  label: string;
  value: number;
  total: number;
  color: string;
  bgColor: string;
  icon: React.ReactNode;
}

function MetricCard({ label, value, total, color, bgColor, icon }: MetricCardProps) {
  const percentage = total > 0 ? Math.round((value / total) * 100) : 0;
  const circumference = 2 * Math.PI * 45;
  const strokeDashoffset = circumference - (percentage / 100) * circumference;

  return (
    <div className="bg-white rounded-xl shadow-sm border border-gray-100 p-6 hover:shadow-md transition-all duration-200 group">
      <div className="flex items-center justify-between">
        <div className="flex-1">
          <div className="flex items-center mb-3">
            <div className={`w-10 h-10 ${bgColor} rounded-lg flex items-center justify-center mr-3 group-hover:scale-110 transition-transform duration-200`}>
              {icon}
            </div>
            <h3 className="text-sm font-semibold text-gray-600 uppercase tracking-wider">{label}</h3>
          </div>
          <div className="mb-4">
            <p className="text-3xl font-bold text-gray-900 mb-1">{value.toLocaleString()}</p>
            <p className="text-sm text-gray-500">{percentage}% of total</p>
          </div>
          
          {/* Progress bar */}
          <div className="w-full bg-gray-200 rounded-full h-2">
            <div 
              className={`h-2 rounded-full transition-all duration-500 ease-out ${color}`}
              style={{ width: `${percentage}%` }}
            ></div>
          </div>
        </div>
        
        {/* Circular progress indicator */}
        <div className="relative w-20 h-20">
          <svg className="w-20 h-20 transform -rotate-90" viewBox="0 0 100 100">
            <circle
              cx="50"
              cy="50"
              r="45"
              stroke="currentColor"
              strokeWidth="6"
              fill="transparent"
              className="text-gray-200"
            />
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
              className={`${color} transition-all duration-700 ease-out`}
            />
          </svg>
          <div className="absolute inset-0 flex items-center justify-center">
            <span className="text-lg font-bold text-gray-700">{percentage}%</span>
          </div>
        </div>
      </div>
    </div>
  );
}

const MATERIAL_CONFIG: Record<string, { color: string; bgColor: string; icon: React.ReactNode }> = {
  cardboard: {
    color: 'text-amber-500',
    bgColor: 'bg-amber-50',
    icon: (
      <svg className="w-5 h-5 text-amber-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M20 7l-8-4-8 4m16 0l-8 4m8-4v10l-8 4m0-10L4 7m8 4v10M4 7v10l8 4" />
      </svg>
    )
  },
  glass: {
    color: 'text-green-500',
    bgColor: 'bg-green-50',
    icon: (
      <svg className="w-5 h-5 text-green-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19.428 15.428a2 2 0 00-1.022-.547l-2.387-.477a6 6 0 00-3.86.517l-.318.158a6 6 0 01-3.86.517L6.05 15.21a2 2 0 00-1.806.547M8 4h8l-1 1v5.172a2 2 0 00.586 1.414l5 5c1.26 1.26.367 3.414-1.415 3.414H4.828c-1.782 0-2.674-2.154-1.414-3.414l5-5A2 2 0 009 10.172V5L8 4z" />
      </svg>
    )
  },
  metal: {
    color: 'text-gray-500',
    bgColor: 'bg-gray-50',
    icon: (
      <svg className="w-5 h-5 text-gray-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
      </svg>
    )
  },
  plastic: {
    color: 'text-blue-500',
    bgColor: 'bg-blue-50',
    icon: (
      <svg className="w-5 h-5 text-blue-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 11H5m14 0a2 2 0 012 2v6a2 2 0 01-2 2H5a2 2 0 01-2-2v-6a2 2 0 012-2m14 0V9a2 2 0 00-2-2M5 11V9a2 2 0 012-2m0 0V5a2 2 0 012-2h6a2 2 0 012 2v2M7 7h10" />
      </svg>
    )
  },
  trash: {
    color: 'text-red-500',
    bgColor: 'bg-red-50',
    icon: (
      <svg className="w-5 h-5 text-red-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 7l-.867 12.142A2 2 0 0116.138 21H7.862a2 2 0 01-1.995-1.858L5 7m5 4v6m4-6v6m1-10V4a1 1 0 00-1-1h-4a1 1 0 00-1 1v3M4 7h16" />
      </svg>
    )
  },
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
      <div className="bg-white rounded-xl shadow-sm border border-gray-100 p-6">
        <div className="flex items-center justify-between mb-6">
          <div>
            <h2 className="text-xl font-bold text-gray-900">Classification Metrics</h2>
            <p className="text-gray-600 text-sm mt-1">Real-time sorting statistics</p>
          </div>
          {error ? (
            <span className="inline-flex items-center px-3 py-1 rounded-full text-sm font-medium bg-red-100 text-red-800">
              <div className="w-2 h-2 bg-red-500 rounded-full mr-2"></div>
              {error}
            </span>
          ) : (
            <span className="inline-flex items-center px-3 py-1 rounded-full text-sm font-medium bg-green-100 text-green-800">
              <div className="w-2 h-2 bg-green-500 rounded-full mr-2 animate-pulse"></div>
              Live
            </span>
          )}
        </div>

        {Object.keys(counters).length === 0 ? (
          <div className="text-center py-12">
            <div className="w-20 h-20 mx-auto mb-4 bg-gray-100 rounded-full flex items-center justify-center">
              <svg className="w-10 h-10 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 19v-6a2 2 0 00-2-2H5a2 2 0 00-2 2v6a2 2 0 002 2h2a2 2 0 002-2zm0 0V9a2 2 0 012-2h2a2 2 0 012 2v10m-6 0a2 2 0 002 2h2a2 2 0 002-2m0 0V5a2 2 0 012-2h2a2 2 0 012 2v14a2 2 0 01-2 2h-2a2 2 0 01-2-2z" />
              </svg>
            </div>
            <p className="text-gray-600 font-semibold text-lg">No classifications yet</p>
            <p className="text-gray-500 text-sm mt-1">Start the classifier to see metrics</p>
          </div>
        ) : (
          <div className="grid grid-cols-1 gap-4">
            {Object.entries(counters).map(([label, count]) => {
              const config = MATERIAL_CONFIG[label.toLowerCase()] || {
                color: 'text-gray-500',
                bgColor: 'bg-gray-50',
                icon: (
                  <svg className="w-5 h-5 text-gray-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M7 7h.01M7 3h5c.512 0 1.024.195 1.414.586l7 7a2 2 0 010 2.828l-7 7a2 2 0 01-2.828 0l-7-7A1.994 1.994 0 013 12V7a4 4 0 014-4z" />
                  </svg>
                )
              };
              
              return (
                <MetricCard
                  key={label}
                  label={label}
                  value={count}
                  total={total}
                  color={config.color}
                  bgColor={config.bgColor}
                  icon={config.icon}
                />
              );
            })}
          </div>
        )}

        {total > 0 && (
          <div className="mt-6 pt-6 border-t border-gray-200">
            <div className="bg-gradient-to-r from-blue-50 to-indigo-50 rounded-xl p-6 border border-blue-100">
              <div className="flex items-center justify-between">
                <div className="flex items-center">
                  <div className="w-12 h-12 bg-blue-100 rounded-lg flex items-center justify-center mr-4">
                    <svg className="w-6 h-6 text-blue-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 19v-6a2 2 0 00-2-2H5a2 2 0 00-2 2v6a2 2 0 002 2h2a2 2 0 002-2zm0 0V9a2 2 0 012-2h2a2 2 0 012 2v10m-6 0a2 2 0 002 2h2a2 2 0 002-2m0 0V5a2 2 0 012-2h2a2 2 0 012 2v14a2 2 0 01-2 2h-2a2 2 0 01-2-2z" />
                    </svg>
                  </div>
                  <div>
                    <span className="text-lg font-semibold text-blue-900">Total Classifications</span>
                    <p className="text-blue-600 text-sm">All materials combined</p>
                  </div>
                </div>
                <span className="text-3xl font-bold text-blue-900">{total.toLocaleString()}</span>
              </div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}

export default Counters;