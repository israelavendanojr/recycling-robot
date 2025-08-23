import { useEffect, useState } from 'react';
import { fetchHealth, startClassifier, stopClassifier } from '../api/client';

interface ControlButtonProps {
  onClick: () => void;
  disabled: boolean;
  loading: boolean;
  variant: 'primary' | 'danger' | 'secondary' | 'success';
  icon: React.ReactNode;
  children: React.ReactNode;
}

function ControlButton({ onClick, disabled, loading, variant, icon, children }: ControlButtonProps) {
  const baseStyles = "flex items-center justify-center px-4 py-3 rounded-lg font-medium transition-all duration-200 focus:outline-none focus:ring-2 focus:ring-offset-2 disabled:opacity-50 disabled:cursor-not-allowed";
  
  const variants = {
    primary: "bg-blue-600 hover:bg-blue-700 text-white focus:ring-blue-500 shadow-md hover:shadow-lg",
    danger: "bg-red-600 hover:bg-red-700 text-white focus:ring-red-500 shadow-md hover:shadow-lg",
    secondary: "bg-gray-600 hover:bg-gray-700 text-white focus:ring-gray-500 shadow-md hover:shadow-lg",
    success: "bg-green-600 hover:bg-green-700 text-white focus:ring-green-500 shadow-md hover:shadow-lg"
  };

  return (
    <button
      onClick={onClick}
      disabled={disabled}
      className={`${baseStyles} ${variants[variant]}`}
    >
      <span className="mr-2">
        {loading ? (
          <div className="animate-spin rounded-full h-5 w-5 border-b-2 border-white"></div>
        ) : (
          icon
        )}
      </span>
      {children}
    </button>
  );
}

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
    <div className="bg-white rounded-xl shadow-lg p-6">
      <div className="flex items-center justify-between mb-6">
        <h2 className="text-xl font-semibold text-gray-900">System Controls</h2>
        <div className={`flex items-center px-3 py-1 rounded-full text-sm font-medium ${
          isHealthy ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'
        }`}>
          <div className={`w-2 h-2 rounded-full mr-2 ${
            isHealthy ? 'bg-green-500' : 'bg-red-500'
          }`}></div>
          {isHealthy ? 'System Healthy' : 'System Issues'}
        </div>
      </div>

      {error && (
        <div className="mb-4 p-3 bg-red-50 border border-red-200 rounded-lg">
          <p className="text-sm text-red-700">{error}</p>
        </div>
      )}

      <div className="grid grid-cols-1 sm:grid-cols-2 gap-3">
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
        <div className="mt-6 pt-6 border-t border-gray-200">
          <h3 className="text-sm font-medium text-gray-700 mb-3">System Status</h3>
          <div className="grid grid-cols-3 gap-4">
            {Object.entries(status.services || {}).map(([service, healthy]) => (
              <div key={service} className="text-center">
                <div className={`w-8 h-8 mx-auto mb-1 rounded-full flex items-center justify-center ${
                  healthy ? 'bg-green-100' : 'bg-red-100'
                }`}>
                  {healthy ? (
                    <svg className="w-4 h-4 text-green-600" fill="currentColor" viewBox="0 0 20 20">
                      <path fillRule="evenodd" d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z" clipRule="evenodd" />
                    </svg>
                  ) : (
                    <svg className="w-4 h-4 text-red-600" fill="currentColor" viewBox="0 0 20 20">
                      <path fillRule="evenodd" d="M4.293 4.293a1 1 0 011.414 0L10 8.586l4.293-4.293a1 1 0 111.414 1.414L11.414 10l4.293 4.293a1 1 0 01-1.414 1.414L10 11.414l-4.293 4.293a1 1 0 01-1.414-1.414L8.586 10 4.293 5.707a1 1 0 010-1.414z" clipRule="evenodd" />
                    </svg>
                  )}
                </div>
                <p className="text-xs text-gray-600 capitalize">{service}</p>
              </div>
            ))}
          </div>
        </div>
      )}
    </div>
  );
}

export default Controls;