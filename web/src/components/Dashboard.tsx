import VideoFeed from './VideoFeed';
import ClassificationLog from './ClassificationLog';

export function Dashboard() {
  return (
    <div className="min-h-screen bg-gray-50 p-6">
      <div className="max-w-7xl mx-auto">
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
