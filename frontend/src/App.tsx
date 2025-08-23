import VideoFeed from './components/VideoFeed'
import ClassificationLog from './components/ClassificationLog'
import Counters from './components/Counters'
import Controls from './components/Controls'

function App() {
  return (
    <div className="min-h-screen bg-gray-50 p-6">
      <div className="max-w-7xl mx-auto space-y-6">
        <header className="mb-8">
          <h1 className="text-3xl font-bold text-gray-900 mb-2">Recycling Classification Dashboard</h1>
          <p className="text-gray-600">Real-time object detection and sorting metrics</p>
        </header>
        
        <div className="grid grid-cols-1 xl:grid-cols-3 gap-6">
          {/* Video Feed - Takes up 2 columns on desktop */}
          <div className="xl:col-span-2 space-y-6">
            <VideoFeed />
            <ClassificationLog />
          </div>
          
          {/* Metrics and Controls sidebar */}
          <div className="space-y-6">
            <Counters />
            <Controls />
          </div>
        </div>
      </div>
    </div>
  )
}

export default App
