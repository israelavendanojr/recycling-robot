import { API_BASE } from '../config';

export type ClassificationEvent = {
  id: number;
  class: string;
  confidence: number;
  timestamp: number;
  raw_logits?: string;
  image_source?: string;
  created_at?: string;
};

export async function fetchEvents(signal?: AbortSignal): Promise<ClassificationEvent[]> {
  console.log('[API] Fetching events from:', `${API_BASE}/api/classifications`);
  
  try {
    const res = await fetch(`${API_BASE}/api/classifications`, { signal });
    if (!res.ok) {
      throw new Error(`Failed to fetch events: ${res.status} ${res.statusText}`);
    }
    
    const data = await res.json();
    console.log('[API] Received classifications response:', data);
    
    if (data.success && data.classifications) {
      // Transform the data to match the expected format
      const events = data.classifications.map((item: any) => ({
        id: item.id,
        class: item.label,
        confidence: item.confidence,
        timestamp: new Date(item.timestamp).getTime() / 1000, // Convert to Unix timestamp
        raw_logits: item.raw_logits,
        image_source: item.image_source,
        created_at: item.created_at
      }));
      
      console.log(`[API] Successfully fetched ${events.length} classifications`);
      return events;
    } else if (data.classifications) {
      // Handle case where success field might be missing
      const events = data.classifications.map((item: any) => ({
        id: item.id,
        class: item.label,
        confidence: item.confidence,
        timestamp: new Date(item.timestamp).getTime() / 1000,
        raw_logits: item.raw_logits,
        image_source: item.image_source,
        created_at: item.created_at
      }));
      
      console.log(`[API] Successfully fetched ${events.length} classifications (no success field)`);
      return events;
    } else {
      console.warn('[API] Unexpected response format:', data);
      return [];
    }
  } catch (e) {
    console.error('[API] Failed to fetch events:', e);
    throw e;
  }
}

export async function fetchCounters(signal?: AbortSignal): Promise<Record<string, number>> {
  console.log('[API] Fetching counters from:', `${API_BASE}/api/counters`);
  
  try {
    const res = await fetch(`${API_BASE}/api/counters`, { signal });
    if (!res.ok) throw new Error(`Failed to fetch counters: ${res.status}`);
    
    const data = await res.json();
    console.log('[API] Received counters:', data);
    return data;
  } catch (e) {
    console.error('[API] Failed to fetch counters:', e);
    throw e;
  }
}

export async function fetchHealth(signal?: AbortSignal): Promise<any> {
  console.log('[API] Fetching health from:', `${API_BASE}/api/health`);
  
  try {
    const res = await fetch(`${API_BASE}/api/health`, { signal });
    if (!res.ok) throw new Error(`Failed to fetch health: ${res.status}`);
    
    const data = await res.json();
    console.log('[API] Received health status:', data);
    return data;
  } catch (e) {
    console.error('[API] Failed to fetch health:', e);
    throw e;
  }
}

export async function fetchLatestClassification(signal?: AbortSignal): Promise<ClassificationEvent | null> {
  console.log('[API] Fetching latest classification from:', `${API_BASE}/api/classifications/latest`);
  
  try {
    const res = await fetch(`${API_BASE}/api/classifications/latest`, { signal });
    if (!res.ok) {
      throw new Error(`Failed to fetch latest classification: ${res.status} ${res.statusText}`);
    }
    
    const data = await res.json();
    console.log('[API] Received latest classification response:', data);
    
    if (data.success && data.classification) {
      const item = data.classification;
      const event = {
        id: item.id,
        class: item.label,
        confidence: item.confidence,
        timestamp: new Date(item.timestamp).getTime() / 1000,
        raw_logits: item.raw_logits,
        image_source: item.image_source,
        created_at: item.created_at
      };
      
      console.log('[API] Successfully fetched latest classification:', event);
      return event;
    } else {
      console.warn('[API] No latest classification found or unexpected format:', data);
      return null;
    }
  } catch (e) {
    console.error('[API] Failed to fetch latest classification:', e);
    return null;
  }
}

export async function fetchCurrentImage(signal?: AbortSignal): Promise<{ image_url: string; timestamp: number; source: string } | null> {
  console.log('[API] Fetching current image info from:', `${API_BASE}/api/current_image`);
  
  try {
    const res = await fetch(`${API_BASE}/api/current_image`, { signal });
    if (!res.ok) {
      throw new Error(`Failed to fetch current image: ${res.status} ${res.statusText}`);
    }
    
    const data = await res.json();
    console.log('[API] Received current image response:', data);
    
    if (data.success && data.image_url) {
      const imageInfo = {
        image_url: `${API_BASE}${data.image_url}`,
        timestamp: data.timestamp,
        source: data.source
      };
      
      console.log('[API] Successfully fetched current image info:', imageInfo);
      return imageInfo;
    } else {
      console.warn('[API] No current image found or unexpected format:', data);
      return null;
    }
  } catch (e) {
    console.error('[API] Failed to fetch current image:', e);
    return null;
  }
}

export async function startClassifier(): Promise<{ success: boolean; running: boolean }>{
  console.log('[API] Starting classifier...');
  
  try {
    const res = await fetch(`${API_BASE}/api/classifier/start`, { method: 'POST' });
    if (!res.ok) throw new Error(`Failed to start classifier: ${res.status}`);
    
    const data = await res.json();
    console.log('[API] Classifier start response:', data);
    return data;
  } catch (e) {
    console.error('[API] Failed to start classifier:', e);
    throw e;
  }
}

export async function stopClassifier(): Promise<{ success: boolean; running: boolean }>{
  console.log('[API] Stopping classifier...');
  
  try {
    const res = await fetch(`${API_BASE}/api/classifier/stop`, { method: 'POST' });
    if (!res.ok) throw new Error(`Failed to stop classifier: ${res.status}`);
    
    const data = await res.json();
    console.log('[API] Classifier stop response:', data);
    return data;
  } catch (e) {
    console.error('[API] Failed to stop classifier:', e);
    throw e;
  }
}

