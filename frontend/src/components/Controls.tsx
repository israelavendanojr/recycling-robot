import { useEffect, useState } from 'react';
import { fetchHealth, startClassifier, stopClassifier } from '../api/client';

export function Controls() {
  const [loading, setLoading] = useState<'start' | 'stop' | null>(null);
  const [status, setStatus] = useState<any>(null);
  const [error, setError] = useState<string | null>(null);

  async function refreshHealth() {
    try {
      const res = await fetchHealth();
      setStatus(res);
    } catch (e: any) {
      console.error('[Controls] fetchHealth failed', e);
      setError(e?.message || 'Health check failed');
    }
  }

  useEffect(() => {
    refreshHealth();
    const timer = setInterval(refreshHealth, 5000);
    return () => clearInterval(timer);
  }, []);

  async function onStart() {
    setLoading('start');
    setError(null);
    try {
      await startClassifier();
      await refreshHealth();
    } catch (e: any) {
      console.error('[Controls] startClassifier failed', e);
      setError(e?.message || 'Failed to start');
    } finally {
      setLoading(null);
    }
  }

  async function onStop() {
    setLoading('stop');
    setError(null);
    try {
      await stopClassifier();
      await refreshHealth();
    } catch (e: any) {
      console.error('[Controls] stopClassifier failed', e);
      setError(e?.message || 'Failed to stop');
    } finally {
      setLoading(null);
    }
  }

  const ok = status?.ok;

  return (
    <div className="bg-white rounded-lg shadow p-4">
      <div className="flex items-center justify-between mb-3">
        <h3 className="font-semibold">Controls</h3>
        <span className={`px-2 py-1 rounded text-xs ${ok ? 'bg-green-100 text-green-700' : 'bg-red-100 text-red-700'}`}>
          {ok ? 'Healthy' : 'Degraded'}
        </span>
      </div>
      {error && <div className="text-sm text-red-600 mb-2">{error}</div>}
      <div className="flex gap-2">
        <button onClick={onStart} disabled={loading!==null} className="px-3 py-2 bg-blue-600 text-white rounded disabled:opacity-50">
          {loading==='start' ? 'Starting...' : 'Start Classifier'}
        </button>
        <button onClick={onStop} disabled={loading!==null} className="px-3 py-2 bg-gray-700 text-white rounded disabled:opacity-50">
          {loading==='stop' ? 'Stopping...' : 'Stop Classifier'}
        </button>
      </div>
    </div>
  );
}

export default Controls;

