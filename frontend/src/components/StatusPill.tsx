import React, { useState, useEffect } from 'react';

interface HealthStatus {
  ok: boolean;
  services: {
    ros2: boolean;
    camera: boolean;
    db: boolean;
  };
}

export function StatusPill() {
  const [status, setStatus] = useState<HealthStatus | null>(null);

  useEffect(() => {
    const fetchStatus = async () => {
      try {
        const response = await fetch('/api/health');
        const data = await response.json();
        setStatus(data);
      } catch (error) {
        setStatus({ ok: false, services: { ros2: false, camera: false, db: false } });
      }
    };

    fetchStatus();
    const interval = setInterval(fetchStatus, 5000);
    return () => clearInterval(interval);
  }, []);

  if (!status) return <div className="animate-pulse bg-gray-300 h-6 w-16 rounded-full" />;

  return (
    <div className={`flex items-center space-x-2 px-3 py-1 rounded-full text-sm ${
      status.ok ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'
    }`}>
      <div className={`w-2 h-2 rounded-full ${
        status.ok ? 'bg-green-500' : 'bg-red-500'
      }`} />
      <span>{status.ok ? 'Online' : 'Issues'}</span>
    </div>
  );
}