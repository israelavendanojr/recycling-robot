import type { Config } from 'tailwindcss'

const config: Config = {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        brand: {
          bg: {
            primary: '#f8fafc',
            secondary: '#f1f5f9',
          },
          card: '#ffffff',
          border: '#e5e7eb',
          text: {
            primary: '#1f2937',
            secondary: '#6b7280',
            label: '#374151',
          },
          teal: {
            'dark-bar': '#0e7490',
            medium: '#0891b2',
            mid: '#06b6d4',
            light: '#67e8f9',
            accent: '#a5f3fc',
          },
        },
      },
      fontFamily: {
        sans: [
          'Inter',
          '-apple-system',
          'BlinkMacSystemFont',
          'Segoe UI',
          'Roboto',
          'sans-serif',
        ],
      },
    },
  },
  plugins: [],
}

export default config
