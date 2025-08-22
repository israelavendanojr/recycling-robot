import { useEffect, useRef, useState } from 'react';
import { fetchCounters } from '../api/client';

function sum(values: number[]): number { return values.reduce((a, b) => a + b, 0); }

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
                <span>{count} ({pct}%)</span>
              </div>
              <div className="w-full bg-gray-100 h-2 rounded">
                <div className="bg-green-500 h-2 rounded" style={{ width: `${pct}%` }} />
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
import { useState, useEffect } from 'react';

export function Counters() {
  const [counters, setCounters] = useState<Record<string, number>>({});

  useEffect(() => {
    const fetchCounters = async () => {
      try {
        const response = await fetch('/api/counters');
        const data = await response.json();
        setCounters(data);
      } catch (error) {
        console.error('Failed to fetch counters:', error);
      }
    };

    fetchCounters();
    const interval = setInterval(fetchCounters, 3000);
    return () => clearInterval(interval);
  }, []);

  const total = Object.values(counters).reduce((sum, count) => sum + count, 0);

  return (
    <div className="bg-white rounded-lg shadow p-6">
      <h2 className="text-xl font-semibold mb-4">Recycling Counters</h2>
      
      <div className="space-y-3">
        {Object.entries(counters).map(([material, count]) => (
          <div key={material} className="flex justify-between items-center">
            <span className="capitalize font-medium">{material}</span>
            <div className="flex items-center space-x-3">
              <div className="bg-gray-200 rounded-full h-2 w-20">
                <div 
                  className="bg-blue-500 h-2 rounded-full"
                  style={{ width: `${total > 0 ? (count / total) * 100 : 0}%` }}
                />
              </div>
              <span className="font-bold text-blue-600">{count}</span>
            </div>
          </div>
        ))}
        
        {Object.keys(counters).length === 0 && (
          <p className="text-gray-500 text-center py-4">No classifications yet</p>
        )}
      </div>
      
      <div className="mt-4 pt-4 border-t">
        <div className="flex justify-between items-center font-semibold">
          <span>Total Items</span>
          <span className="text-lg text-blue-600">{total}</span>
        </div>
      </div>
    </div>
  );
}