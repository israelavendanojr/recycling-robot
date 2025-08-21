import { useState, useEffect } from 'react';

interface Event {
  id: number;
  class: string;
  confidence: number;
  timestamp: number;
}

export function EventsList() {
  const [events, setEvents] = useState<Event[]>([]);

  useEffect(() => {
    const fetchEvents = async () => {
      try {
        const response = await fetch('/api/events');
        const data = await response.json();
        setEvents(data);
      } catch (error) {
        console.error('Failed to fetch events:', error);
      }
    };

    fetchEvents();
    const interval = setInterval(fetchEvents, 5000);
    return () => clearInterval(interval);
  }, []);

  const formatTime = (timestamp: number) => {
    return new Date(timestamp * 1000).toLocaleTimeString();
  };

  return (
    <div className="bg-white rounded-lg shadow p-6">
      <h2 className="text-xl font-semibold mb-4">Recent Classifications</h2>
      
      <div className="space-y-2 max-h-64 overflow-y-auto">
        {events.map((event) => (
          <div key={event.id} className="flex justify-between items-center p-2 bg-gray-50 rounded">
            <div>
              <span className="font-medium capitalize">{event.class}</span>
              <span className="text-sm text-gray-600 ml-2">
                ({(event.confidence * 100).toFixed(1)}%)
              </span>
            </div>
            <span className="text-xs text-gray-500">
              {formatTime(event.timestamp)}
            </span>
          </div>
        ))}
        
        {events.length === 0 && (
          <p className="text-gray-500 text-center py-4">No events yet</p>
        )}
      </div>
    </div>
  );
}