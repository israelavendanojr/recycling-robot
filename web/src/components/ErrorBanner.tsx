import React, { useState } from 'react'

interface ErrorBannerProps {
  title: string
  error: Error | string
  className?: string
}

export const ErrorBanner: React.FC<ErrorBannerProps> = ({ title, error, className = '' }) => {
  const [copied, setCopied] = useState(false)

  const errorMessage = error instanceof Error ? error.message : error
  const errorDetails = error instanceof Error 
    ? JSON.stringify({ message: error.message, stack: error.stack }, null, 2)
    : error

  const handleCopyDetails = async () => {
    try {
      await navigator.clipboard.writeText(errorDetails)
      setCopied(true)
      setTimeout(() => setCopied(false), 2000)
    } catch (err) {
      console.error('Failed to copy error details:', err)
    }
  }

  return (
    <div className={`card bg-red-50 border-red-200 ${className}`}>
      <div className="flex items-start justify-between">
        <div className="flex-1">
          <div className="text-red-600 font-medium">{title}</div>
          <div className="text-red-500 text-sm mt-1">
            {errorMessage.length > 100 
              ? `${errorMessage.substring(0, 100)}...` 
              : errorMessage
            }
          </div>
        </div>
        <button
          onClick={handleCopyDetails}
          className="ml-3 px-3 py-1 text-xs bg-red-100 hover:bg-red-200 text-red-700 rounded transition-colors"
          title="Copy full error details"
        >
          {copied ? '✓ Copied' : 'Copy Details'}
        </button>
      </div>
    </div>
  )
}
