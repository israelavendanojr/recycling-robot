import React from 'react'
import { SystemStatus } from './SystemStatus'
import { ClassificationPanel } from './ClassificationPanel'
import { MaterialCounting } from './MaterialCounting'
import { ConfidenceChart } from './ConfidenceChart'
import { ClassificationLogs } from './ClassificationLogs'
import { SystemMetrics } from './SystemMetrics'

export const Dashboard: React.FC = () => {
  return (
    <div className="min-h-screen bg-brand-bg-primary p-6">
      <div className="max-w-7xl mx-auto space-y-6">
        {/* Header */}
        <div className="text-center py-8">
          <h1 className="text-3xl font-bold text-brand-text-primary mb-2">
            Recycler Dashboard
          </h1>
          <p className="text-brand-text-secondary">
            Monitoring, controls, and analytics
          </p>
        </div>

        {/* System Status */}
        <SystemStatus />

        {/* Main Content Grid */}
        <div className="grid grid-cols-1 xl:grid-cols-2 gap-6">
          {/* Classification Panel */}
          <ClassificationPanel />
          
          {/* Material Counting */}
          <MaterialCounting />
        </div>

        {/* Charts Row */}
        <div className="grid grid-cols-1 gap-6">
          <ConfidenceChart />
        </div>

        {/* Logs */}
        <ClassificationLogs />

        {/* System Metrics */}
        <SystemMetrics />
      </div>
    </div>
  )
}
