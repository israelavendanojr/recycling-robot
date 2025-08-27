import VideoFeed from './VideoFeed';
import ClassificationLog from './ClassificationLog';
import PipelineStatus from './PipelineStatus';

export function Dashboard() {
  return (
    <div className="min-h-screen bg-gray-50 p-6">
      <div className="max-w-7xl mx-auto">
        {/* Header with Pipeline Status */}
        <div className="mb-8">
          <h1 className="text-3xl font-bold text-gray-900 mb-4">Recycling Robot Dashboard</h1>
          <div className="max-w-md">
            <PipelineStatus />
          </div>
        </div>
        
        {/* Two Component Layout */}
        <div className="grid grid-cols-1 xl:grid-cols-2 gap-8">
          {/* Left Column - Camera Feed */}
          <VideoFeed />
          
          {/* Right Column - Classification Logs */}
          <ClassificationLog />
        </div>
      </div>
    </div>
  );
}

export default Dashboard;
