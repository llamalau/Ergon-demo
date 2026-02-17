interface SimulationPlayerProps {
  videoUrl: string | null;
  title?: string;
}

export default function SimulationPlayer({
  videoUrl,
  title = "Simulation",
}: SimulationPlayerProps) {
  if (!videoUrl) {
    return (
      <div className="w-full h-64 bg-gray-900 rounded-lg border border-gray-800 flex items-center justify-center text-gray-600">
        No video available
      </div>
    );
  }

  return (
    <div className="bg-gray-900 border border-gray-800 rounded-lg overflow-hidden">
      <div className="px-4 py-2 border-b border-gray-800">
        <h3 className="text-sm font-medium text-gray-300">{title}</h3>
      </div>
      <video
        src={videoUrl}
        controls
        className="w-full"
        playsInline
      >
        Your browser does not support the video tag.
      </video>
    </div>
  );
}
