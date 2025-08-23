import { useEffect, useState } from 'react';
import { fetchHealth, startClassifier, stopClassifier } from '../api/client';
import ControlButton from './ControlButton';

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

  const isHealthy = status?.ok;

  return (
    <div className="bg-white rounded-xl shadow-sm border border-gray-100 p-6">
      <div className="flex items-center justify-between mb-6">
        <div>
          <h2 className="text-xl font-bold text-gray-900">System Controls</h2>
          <p className="text-gray-600 text-sm mt-1">Manage robot operations</p>
        </div>
        <div className={`flex items-center px-4 py-2 rounded-full text-sm font-semibold transition-all duration-200 ${
          isHealthy ? 'bg-green-100 text-green-800 border border-green-200' : 'bg-red-100 text-red-800 border border-red-200'
        }`}>
          <div className={`w-2 h-2 rounded-full mr-2 ${isHealthy ? 'bg-green-500' : 'bg-red-500'}`}></div>
          {isHealthy ? 'System Healthy' : 'System Issues'}
        </div>
      </div>

      {error && (
        <div className="mb-6 p-4 bg-red-50 border border-red-200 rounded-xl">
          <div className="flex items-center">
            <svg className="w-5 h-5 text-red-500 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4m0 4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
            </svg>
            <p className="text-sm text-red-700 font-medium">{error}</p>
          </div>
        </div>
      )}

      <div className="grid grid-cols-1 sm:grid-cols-2 gap-4 mb-6">
        <ControlButton
          onClick={onStart}
          disabled={loading !== null}
          loading={loading === 'start'}
          variant="primary"
          icon={
            <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M14.828 14.828a4 4 0 01-5.656 0M9 10h1m4 0h1m-6 4h.01M9 16h.01M20 12a8 8 0 11-16 0 8 8 0 0116 0z" />
            </svg>
          }
        >
          Start Classifier
        </ControlButton>

        <ControlButton
          onClick={onStop}
          disabled={loading !== null}
          loading={loading === 'stop'}
          variant="danger"
          icon={
            <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 10h6v4H9z" />
            </svg>
          }
        >
          Stop Classifier
        </ControlButton>

        <ControlButton
          onClick={() => window.location.reload()}
          disabled={loading !== null}
          loading={false}
          variant="secondary"
          icon={
            <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15" />
            </svg>
          }
        >
          Reset System
        </ControlButton>

        <ControlButton
          onClick={refreshHealth}
          disabled={loading !== null}
          loading={false}
          variant="success"
          icon={
            <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" />
            </svg>
          }
        >
          Health Check
        </ControlButton>
      </div>

      {status && (
        <div className="pt-6 border-t border-gray-200">
          <h3 className="text-sm font-semibold text-gray-700 mb-4 uppercase tracking-wider">System Status</h3>
          <div className="grid grid-cols-3 gap-4">
            {Object.entries(status.services || {}).map(([service, healthy]) => (
              <div key={service} className="text-center group">
                <div className={`w-12 h-12 mx-auto mb-2 rounded-xl flex items-center justify-center transition-all duration-200 ${
                  healthy ? 'bg-green-100 group-hover:bg-green-200' : 'bg-red-100 group-hover:bg-red-200'
                }`}>
                  {healthy ? (
                    <svg className="w-6 h-6 text-green-600" fill="currentColor" viewBox="0 0 20 20">
                      <path fillRule="evenodd" d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z" clipRule="evenodd" />
                    </svg>
                  ) : (
                    <svg className="w-6 h-6 text-red-600" fill="currentColor" viewBox="0 0 20 20">
                      <path fillRule="evenodd" d="M4.293 4.293a1 1 0 011.414 0L10 8.586l4.293-4.293a1 1 0 111.414 1.414L11.414 10l4.293 4.293a1 1 0 01-1.414 1.414L10 11.414l-4.293 4.293a1 1 0 01-1.414-1.414L8.586 10 4.293 5.707a1 1 0 010-1.414z" clipRule="evenodd" />
                    </svg>
                  )}
                </div>
                <p className="text-xs font-medium text-gray-600 capitalize">{service}</p>
                <div className={`w-2 h-2 mx-auto mt-1 rounded-full ${healthy ? 'bg-green-500' : 'bg-red-500'}`}></div>
              </div>
            ))}
          </div>
        </div>
      )}
    </div>
  );
}

export default Controls;