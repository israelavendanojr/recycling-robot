import VideoFeed from './components/VideoFeed'
import ClassificationLog from './components/ClassificationLog'
import Counters from './components/Counters'
import Controls from './components/Controls'

function App() {
  return (
    <div className="min-h-screen bg-gray-100 p-4">
      <div className="max-w-6xl mx-auto grid grid-cols-1 lg:grid-cols-3 gap-4">
        <div className="lg:col-span-2 space-y-4">
          <div className="bg-white rounded-lg shadow p-4">
            <h2 className="text-xl font-semibold mb-3">Video</h2>
            <VideoFeed />
          </div>
          <ClassificationLog />
        </div>
        <div className="space-y-4">
          <Counters />
          <Controls />
        </div>
      </div>
    </div>
  )
}

export default App