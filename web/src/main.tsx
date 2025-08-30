import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import App from './App.tsx'

// Enable API debugging with: localStorage.setItem('DEBUG_API','1')
// Disable with: localStorage.removeItem('DEBUG_API')

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <App />
  </StrictMode>,
)
