// src/api/devLog.ts
export const devLog = (...args: unknown[]) => {
  if (import.meta.env.DEV && localStorage.getItem('DEBUG_API') === '1') {
    console.log('[API]', ...args);
  }
};
