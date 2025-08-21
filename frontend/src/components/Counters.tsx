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