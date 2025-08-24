// import React from 'react';
import { useState } from 'react';
import VideoFeed from './VideoFeed';
import ClassificationLog from './ClassificationLog';
import Counters from './Counters';
import Controls from './Controls';
import MetricCard from './MetricCard';
import SystemStatus from './SystemStatus';

// Example dashboard with additional metrics
export function Dashboard() {
  const [cameraStatus, setCameraStatus] = useState<'active' | 'inactive' | 'error'>('inactive');

  const handleCameraStatusChange = (status: 'active' | 'inactive' | 'error') => {
    setCameraStatus(status);
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-50 to-gray-100">
      {/* Professional Header */}
      <header className="bg-white border-b border-gray-200 shadow-sm">
        <div className="max-w-7xl mx-auto px-6 py-6">
          <div className="flex items-center justify-between">
            <div>
              <h1 className="text-3xl font-bold bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">
                Recycling Robot Dashboard
              </h1>
              <p className="text-gray-600 mt-1 text-lg">AI-powered waste classification & sorting system</p>
            </div>
            <div className="flex items-center space-x-4">
              <div className="flex items-center px-4 py-2 bg-green-50 rounded-full border border-green-200">
                <div className="w-2 h-2 bg-green-500 rounded-full mr-2 animate-pulse"></div>
                <span className="text-sm font-medium text-green-700">System Online</span>
              </div>
              <div className="text-right">
                <p className="text-sm text-gray-500">Last Updated</p>
                <p className="text-sm font-medium text-gray-900">{new Date().toLocaleTimeString()}</p>
              </div>
            </div>
          </div>
        </div>
      </header>

      {/* Main Dashboard Content */}
      <main className="max-w-7xl mx-auto px-6 py-8">
        {/* Quick Stats Row */}
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 mb-8">
          <MetricCard
            title="Total Items"
            value="1,247"
            subtitle="Processed today"
            icon={
              <svg className="w-5 h-5 text-blue-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M20 7l-8-4-8 4m16 0l-8 4m8-4v10l-8 4m0-10L4 7m8 4v10M4 7v10l8 4" />
              </svg>
            }
            color="text-blue-600"
            bgColor="bg-blue-50"
            trend={{ value: 12, isPositive: true, label: "vs yesterday" }}
          />
          
          <MetricCard
            title="Accuracy Rate"
            value="94.2%"
            subtitle="AI classification"
            icon={
              <svg className="w-5 h-5 text-green-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" />
              </svg>
            }
            color="text-green-600"
            bgColor="bg-green-50"
            trend={{ value: 2.1, isPositive: true, label: "vs last week" }}
          />
          
          <MetricCard
            title="Processing Speed"
            value="2.3s"
            subtitle="Average per item"
            icon={
              <svg className="w-5 h-5 text-purple-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
              </svg>
            }
            color="text-purple-600"
            bgColor="bg-purple-50"
            trend={{ value: 0.4, isPositive: true, label: "faster" }}
          />
          
          <MetricCard
            title="System Uptime"
            value="99.8%"
            subtitle="This month"
            icon={
              <svg className="w-5 h-5 text-indigo-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z" />
              </svg>
            }
            color="text-indigo-600"
            bgColor="bg-indigo-50"
            trend={{ value: 0.2, isPositive: true, label: "vs target" }}
          />
        </div>

        {/* Main Dashboard Grid */}
        <div className="grid grid-cols-1 xl:grid-cols-3 gap-8">
          {/* Left Column - Video Feed & Classification Log */}
          <div className="xl:col-span-2 space-y-8">
            <VideoFeed onCameraStatusChange={handleCameraStatusChange} />
            <ClassificationLog />
          </div>
          
          {/* Right Column - System Status, Metrics & Controls */}
          <div className="space-y-8">
            <SystemStatus cameraStatus={cameraStatus} />
            <Counters />
            <Controls />
          </div>
        </div>

        {/* Additional Info Section */}
        <div className="mt-12 grid grid-cols-1 lg:grid-cols-2 gap-8">
          <div className="bg-white rounded-xl shadow-sm border border-gray-100 p-6">
            <div className="flex items-center mb-4">
              <div className="w-10 h-10 bg-blue-100 rounded-lg flex items-center justify-center mr-4">
                <svg className="w-6 h-6 text-blue-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 16h-1v-4h-1m1-4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
                </svg>
              </div>
              <h3 className="text-lg font-bold text-gray-900">System Information</h3>
            </div>
            <div className="space-y-3 text-sm text-gray-600">
              <div className="flex justify-between">
                <span>Robot Model:</span>
                <span className="font-medium">RecyclingBot Pro v2.1</span>
              </div>
              <div className="flex justify-between">
                <span>AI Model:</span>
                <span className="font-medium">YOLO v8 + Custom Classifier</span>
              </div>
              <div className="flex justify-between">
                <span>Camera:</span>
                <span className="font-medium">Logitech C270 HD Webcam</span>
              </div>
              <div className="flex justify-between">
                <span>Camera Status:</span>
                <span className={`font-medium ${
                  cameraStatus === 'active' ? 'text-green-600' : 
                  cameraStatus === 'error' ? 'text-red-600' : 'text-yellow-600'
                }`}>
                  {cameraStatus === 'active' ? 'Active' : 
                   cameraStatus === 'error' ? 'Error' : 'Inactive'}
                </span>
              </div>
              <div className="flex justify-between">
                <span>Last Calibration:</span>
                <span className="font-medium">2 days ago</span>
              </div>
            </div>
          </div>

          <div className="bg-white rounded-xl shadow-sm border border-gray-100 p-6">
            <div className="flex items-center mb-4">
              <div className="w-10 h-10 bg-green-100 rounded-lg flex items-center justify-center mr-4">
                <svg className="w-6 h-6 text-green-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 19v-6a2 2 0 00-2-2H5a2 2 0 00-2 2v6a2 2 0 002 2h2a2 2 0 002-2zm0 0V9a2 2 0 012-2h2a2 2 0 012 2v10m-6 0a2 2 0 002 2h2a2 2 0 002-2m0 0V5a2 2 0 012-2h2a2 2 0 012 2v14a2 2 0 01-2 2h-2a2 2 0 01-2-2z" />
                </svg>
              </div>
              <h3 className="text-lg font-bold text-gray-900">Performance Metrics</h3>
            </div>
            <div className="space-y-4">
              <div>
                <div className="flex justify-between text-sm mb-1">
                  <span className="text-gray-600">CPU Usage</span>
                  <span className="font-medium text-gray-900">67%</span>
                </div>
                <div className="w-full bg-gray-200 rounded-full h-2">
                  <div className="bg-blue-600 h-2 rounded-full" style={{ width: '67%' }}></div>
                </div>
              </div>
              <div>
                <div className="flex justify-between text-sm mb-1">
                  <span className="text-gray-600">Memory Usage</span>
                  <span className="font-medium text-gray-900">42%</span>
                </div>
                <div className="w-full bg-gray-200 rounded-full h-2">
                  <div className="bg-green-600 h-2 rounded-full" style={{ width: '42%' }}></div>
                </div>
              </div>
              <div>
                <div className="flex justify-between text-sm mb-1">
                  <span className="text-gray-600">GPU Usage</span>
                  <span className="font-medium text-gray-900">89%</span>
                </div>
                <div className="w-full bg-gray-200 rounded-full h-2">
                  <div className="bg-purple-600 h-2 rounded-full" style={{ width: '89%' }}></div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </main>
    </div>
  );
}

export default Dashboard;
