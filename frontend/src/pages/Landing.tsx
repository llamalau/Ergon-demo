export default function Landing() {
  return (
    <div className="min-h-screen flex flex-col items-center justify-center bg-gray-950">
      <h1 className="text-6xl font-bold text-white mb-4">Ergon</h1>
      <p className="text-xl text-gray-400 mb-8">
        CAD to Manipulation Quality Platform
      </p>
      <div className="flex gap-4">
        <a
          href="/upload"
          className="px-6 py-3 bg-ergon-600 text-white rounded-lg hover:bg-ergon-700 transition-colors font-medium"
        >
          Upload CAD Model
        </a>
        <a
          href="/dashboard"
          className="px-6 py-3 bg-gray-800 text-gray-200 rounded-lg hover:bg-gray-700 transition-colors font-medium"
        >
          View Dashboard
        </a>
      </div>
      <div className="mt-16 grid grid-cols-5 gap-8 text-center text-sm text-gray-500 max-w-4xl">
        {[
          "CAD Ingestion",
          "Digital Twin",
          "Simulation",
          "Analysis",
          "Report",
        ].map((stage, i) => (
          <div key={stage} className="flex flex-col items-center gap-2">
            <div className="w-10 h-10 rounded-full bg-gray-800 flex items-center justify-center text-gray-400 font-mono">
              {i + 1}
            </div>
            <span>{stage}</span>
          </div>
        ))}
      </div>
    </div>
  );
}
