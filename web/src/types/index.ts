export interface SystemHealth {
  status: 'ok' | 'error';
  services: { ros2: boolean; camera: boolean; db: boolean; api?: boolean; };
  system: { cpu_percent: number; memory_percent: number; disk_usage: number; uptime: number; };
  timestamp: number;
  classifier_running: boolean;
}

export interface Classification {
  id: number;
  timestamp: number;
  label: string;
  confidence: number;
  raw_logits?: string | null;
  image_source: string;
  created_at: string;
}

export interface MaterialCounts {
  cardboard: number;
  glass: number;
  metal: number;
  plastic: number;
  trash: number;
}

export interface CurrentImageInfo {
  ts: number;
}
