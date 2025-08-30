export const COLORS = {
  bg: { primary: '#f8fafc', secondary: '#f1f5f9' },
  card: '#ffffff',
  border: '#e5e7eb',
  text: { primary: '#1f2937', secondary: '#6b7280', label: '#374151' },
  teal: { 
    darkBar: '#0e7490', 
    medium: '#0891b2', 
    mid: '#06b6d4', 
    light: '#67e8f9', 
    accent: '#a5f3fc' 
  },
} as const

export const RADII = { card: '0.5rem' } as const        // matches Tailwind rounded-lg
export const SHADOWS = { card: 'shadow-sm' } as const
export const GAP = 6 // Tailwind gap-6 == 24px
