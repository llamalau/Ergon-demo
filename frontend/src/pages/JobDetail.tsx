import { useEffect, useState } from "react";
import { useParams, Link } from "react-router-dom";
import AppShell from "../components/AppShell";
import PipelineTracker from "../components/PipelineTracker";
import LogStream from "../components/LogStream";
import SimulationPlayer from "../components/SimulationPlayer";
import { apiGet, getJobVideos, type Job, type VideoInfo } from "../lib/api";
import { useJobProgress } from "../hooks/useJobProgress";

export default function JobDetail() {
  const { jobId } = useParams<{ jobId: string }>();
  const [job, setJob] = useState<Job | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [videos, setVideos] = useState<VideoInfo[]>([]);
  const progressEvents = useJobProgress(jobId || null);

  useEffect(() => {
    if (!jobId) return;
    const poll = async () => {
      try {
        const data = await apiGet<Job>(`/jobs/${jobId}`);
        setJob(data);
      } catch (e) {
        setError(e instanceof Error ? e.message : "Failed to load job");
      }
    };
    poll();
    const interval = setInterval(poll, 3000);
    return () => clearInterval(interval);
  }, [jobId]);

  // Fetch videos when simulation stage completes
  useEffect(() => {
    if (!jobId || !job) return;
    const simStage = job.stages.find((s) => s.stage_name === "simulation");
    if (simStage?.status === "completed") {
      getJobVideos(jobId)
        .then(setVideos)
        .catch(() => {}); // silently ignore if no videos
    }
  }, [jobId, job?.stages]);

  return (
    <AppShell>
      <div className="max-w-3xl mx-auto">
        <div className="flex items-center justify-between mb-8">
          <h1 className="text-3xl font-bold text-white">Job Details</h1>
          {job?.status === "completed" && (
            <Link
              to={`/reports/${jobId}`}
              className="px-4 py-2 bg-ergon-600 text-white rounded-lg hover:bg-ergon-700 transition-colors text-sm font-medium"
            >
              View Report
            </Link>
          )}
        </div>

        {error && (
          <div className="bg-red-900/30 border border-red-800 rounded-lg p-4 text-red-400 mb-6">
            {error}
          </div>
        )}

        {job && (
          <div className="space-y-6">
            <div className="bg-gray-900 border border-gray-800 rounded-lg p-4 grid grid-cols-3 gap-4 text-sm">
              <div>
                <span className="text-gray-500">Status</span>
                <p className="text-white font-medium capitalize">{job.status}</p>
              </div>
              <div>
                <span className="text-gray-500">Environment</span>
                <p className="text-white font-medium">{job.environment}</p>
              </div>
              <div>
                <span className="text-gray-500">Created</span>
                <p className="text-white font-medium">
                  {new Date(job.created_at).toLocaleString()}
                </p>
              </div>
            </div>

            <PipelineTracker stages={job.stages} />

            {videos.length > 0 && (
              <div>
                <h2 className="text-lg font-semibold text-white mb-4">
                  Simulation Videos
                </h2>
                <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                  {videos.map((v) => (
                    <SimulationPlayer
                      key={v.filename}
                      videoUrl={v.video_url}
                      title={`${v.task_type} — Trial ${v.trial_index + 1}`}
                    />
                  ))}
                </div>
              </div>
            )}

            <div>
              <h2 className="text-lg font-semibold text-white mb-4">Logs</h2>
              <LogStream logs={progressEvents} />
            </div>
          </div>
        )}
      </div>
    </AppShell>
  );
}
