import { API_BASE } from '../config';

export type ClassificationEvent = {
  id: number;
  class: string;
  confidence: number;
  timestamp: number;
};

export async function fetchEvents(signal?: AbortSignal): Promise<ClassificationEvent[]> {
  const res = await fetch(`${API_BASE}/api/events?limit=50`, { signal });
  if (!res.ok) throw new Error(`Failed to fetch events: ${res.status}`);
  return res.json();
}

export async function fetchCounters(signal?: AbortSignal): Promise<Record<string, number>> {
  const res = await fetch(`${API_BASE}/api/counters`, { signal });
  if (!res.ok) throw new Error(`Failed to fetch counters: ${res.status}`);
  return res.json();
}

export async function fetchHealth(signal?: AbortSignal): Promise<any> {
  const res = await fetch(`${API_BASE}/api/health`, { signal });
  if (!res.ok) throw new Error(`Failed to fetch health: ${res.status}`);
  return res.json();
}

export async function startClassifier(): Promise<{ success: boolean; running: boolean }>{
  const res = await fetch(`${API_BASE}/api/classifier/start`, { method: 'POST' });
  if (!res.ok) throw new Error(`Failed to start classifier: ${res.status}`);
  return res.json();
}

export async function stopClassifier(): Promise<{ success: boolean; running: boolean }>{
  const res = await fetch(`${API_BASE}/api/classifier/stop`, { method: 'POST' });
  if (!res.ok) throw new Error(`Failed to stop classifier: ${res.status}`);
  return res.json();
}

