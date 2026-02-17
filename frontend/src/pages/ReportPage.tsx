import { useEffect, useState } from "react";
import { useParams } from "react-router-dom";
import AppShell from "../components/AppShell";
import ScoreCard from "../components/ScoreCard";
import MetricsChart from "../components/MetricsChart";
import RecommendationList from "../components/RecommendationList";
import HeatmapOverlay from "../components/HeatmapOverlay";
import ExportButton from "../components/ExportButton";
import SimulationPlayer from "../components/SimulationPlayer";
import { apiGet, getJobVideos, type Report, type VideoInfo } from "../lib/api";

export default function ReportPage() {
  const { jobId } = useParams<{ jobId: string }>();
  const [report, setReport] = useState<Report | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [videos, setVideos] = useState<VideoInfo[]>([]);

  useEffect(() => {
    if (!jobId) return;
    apiGet<Report>(`/reports/${jobId}`)
      .then(setReport)
      .catch((e) =>
        setError(e instanceof Error ? e.message : "Failed to load report")
      );
    getJobVideos(jobId)
      .then(setVideos)
      .catch(() => {});
  }, [jobId]);

  if (error) {
    return (
      <AppShell>
        <div className="max-w-4xl mx-auto">
          <div className="bg-red-900/30 border border-red-800 rounded-lg p-4 text-red-400">
            {error}
          </div>
        </div>
      </AppShell>
    );
  }

  if (!report) {
    return (
      <AppShell>
        <div className="max-w-4xl mx-auto text-gray-500">Loading report...</div>
      </AppShell>
    );
  }

  // Show the first video as a summary, or null if none available
  const summaryVideo = videos.length > 0 ? videos[0] : null;

  return (
    <AppShell>
      <div className="max-w-4xl mx-auto">
        <div className="flex items-center justify-between mb-8">
          <h1 className="text-3xl font-bold text-white">
            Manipulation Quality Report
          </h1>
          {jobId && <ExportButton jobId={jobId} />}
        </div>

        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6 mb-8">
          <ScoreCard
            score={report.overall_score}
            label="Overall Score"
            size="lg"
          />
          <div className="lg:col-span-2">
            <MetricsChart subScores={report.sub_scores} />
          </div>
        </div>

        <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-8">
          {Object.entries(report.sub_scores).map(([name, score]) => (
            <ScoreCard
              key={name}
              score={score}
              label={name.replace(/_/g, " ")}
              size="sm"
            />
          ))}
        </div>

        {summaryVideo && (
          <div className="mb-8">
            <h2 className="text-lg font-semibold text-white mb-4">
              Simulation Preview
            </h2>
            <SimulationPlayer
              videoUrl={summaryVideo.video_url}
              title={`${summaryVideo.task_type} — Trial ${summaryVideo.trial_index + 1}`}
            />
          </div>
        )}

        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-8">
          <HeatmapOverlay glbUrl={null} />
          <RecommendationList recommendations={report.recommendations as any} />
        </div>

        <div className="bg-gray-900 border border-gray-800 rounded-lg p-6">
          <h3 className="text-sm font-semibold text-gray-300 mb-4">
            Detailed Metrics
          </h3>
          <div className="space-y-4">
            {Object.entries(report.metrics).map(([name, data]) => (
              <div key={name}>
                <h4 className="text-xs text-gray-500 uppercase mb-2">
                  {name.replace(/_/g, " ")}
                </h4>
                <div className="grid grid-cols-2 md:grid-cols-4 gap-2">
                  {Object.entries(data as Record<string, unknown>)
                    .filter(
                      ([k]) =>
                        !["by_task", "details", "categories", "name"].includes(k)
                    )
                    .map(([k, v]) => (
                      <div
                        key={k}
                        className="bg-gray-800 rounded px-3 py-2"
                      >
                        <div className="text-xs text-gray-500">
                          {k.replace(/_/g, " ")}
                        </div>
                        <div className="text-sm text-gray-200 font-mono">
                          {typeof v === "number" ? v.toFixed(2) : String(v)}
                        </div>
                      </div>
                    ))}
                </div>
              </div>
            ))}
          </div>
        </div>
      </div>
    </AppShell>
  );
}
