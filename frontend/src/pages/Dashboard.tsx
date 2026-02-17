import { useEffect, useState } from "react";
import { Link } from "react-router-dom";
import AppShell from "../components/AppShell";
import { apiGet, type Job } from "../lib/api";
import {
  Activity,
  CheckCircle,
  Clock,
  AlertTriangle,
  Server,
} from "lucide-react";

interface HealthCheck {
  status: string;
  checks: Record<string, string>;
}

export default function Dashboard() {
  const [jobs, setJobs] = useState<Job[]>([]);
  const [health, setHealth] = useState<HealthCheck | null>(null);

  useEffect(() => {
    const load = () => {
      apiGet<Job[]>("/jobs").then(setJobs).catch(console.error);
      fetch("/health")
        .then((r) => r.json())
        .then(setHealth)
        .catch(console.error);
    };
    load();
    const interval = setInterval(load, 10000);
    return () => clearInterval(interval);
  }, []);

  const activeJobs = jobs.filter((j) => j.status === "running");
  const completedJobs = jobs.filter((j) => j.status === "completed");
  const failedJobs = jobs.filter((j) => j.status === "failed");

  const statusColor: Record<string, string> = {
    pending: "text-gray-400",
    running: "text-ergon-400",
    completed: "text-green-400",
    failed: "text-red-400",
  };

  return (
    <AppShell>
      <div className="max-w-5xl mx-auto">
        <div className="flex items-center justify-between mb-8">
          <h1 className="text-3xl font-bold text-white">Dashboard</h1>
          <Link
            to="/upload"
            className="px-4 py-2 bg-ergon-600 text-white rounded-lg hover:bg-ergon-700 transition-colors text-sm font-medium"
          >
            New Upload
          </Link>
        </div>

        {/* Stats cards */}
        <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-8">
          <div className="bg-gray-900 border border-gray-800 rounded-lg p-4">
            <div className="flex items-center gap-2 text-gray-500 text-xs mb-2">
              <Activity size={14} />
              Active Jobs
            </div>
            <div className="text-2xl font-bold text-ergon-400">
              {activeJobs.length}
            </div>
          </div>
          <div className="bg-gray-900 border border-gray-800 rounded-lg p-4">
            <div className="flex items-center gap-2 text-gray-500 text-xs mb-2">
              <CheckCircle size={14} />
              Completed
            </div>
            <div className="text-2xl font-bold text-green-400">
              {completedJobs.length}
            </div>
          </div>
          <div className="bg-gray-900 border border-gray-800 rounded-lg p-4">
            <div className="flex items-center gap-2 text-gray-500 text-xs mb-2">
              <AlertTriangle size={14} />
              Failed
            </div>
            <div className="text-2xl font-bold text-red-400">
              {failedJobs.length}
            </div>
          </div>
          <div className="bg-gray-900 border border-gray-800 rounded-lg p-4">
            <div className="flex items-center gap-2 text-gray-500 text-xs mb-2">
              <Server size={14} />
              System Health
            </div>
            <div
              className={`text-2xl font-bold ${
                health?.status === "healthy"
                  ? "text-green-400"
                  : "text-yellow-400"
              }`}
            >
              {health?.status || "..."}
            </div>
          </div>
        </div>

        {/* Health indicators */}
        {health?.checks && (
          <div className="flex gap-3 mb-8">
            {Object.entries(health.checks).map(([service, status]) => (
              <div
                key={service}
                className="flex items-center gap-2 px-3 py-1.5 bg-gray-900 border border-gray-800 rounded-full text-xs"
              >
                <span
                  className={`w-2 h-2 rounded-full ${
                    status === "ok" ? "bg-green-400" : "bg-red-400"
                  }`}
                />
                <span className="text-gray-400 capitalize">{service}</span>
              </div>
            ))}
          </div>
        )}

        {/* Jobs table */}
        <div className="bg-gray-900 border border-gray-800 rounded-lg overflow-hidden">
          <div className="px-4 py-3 border-b border-gray-800">
            <h2 className="text-sm font-semibold text-gray-300">
              Recent Jobs
            </h2>
          </div>
          <table className="w-full text-sm">
            <thead>
              <tr className="border-b border-gray-800 text-gray-500 text-left">
                <th className="p-4">Job ID</th>
                <th className="p-4">Status</th>
                <th className="p-4">Environment</th>
                <th className="p-4">Created</th>
                <th className="p-4">Progress</th>
                <th className="p-4"></th>
              </tr>
            </thead>
            <tbody>
              {jobs.length === 0 && (
                <tr>
                  <td colSpan={6} className="p-8 text-center text-gray-600">
                    No jobs yet. Upload a CAD model to get started.
                  </td>
                </tr>
              )}
              {jobs.map((job) => {
                const completed = job.stages.filter(
                  (s) => s.status === "completed"
                ).length;
                return (
                  <tr
                    key={job.id}
                    className="border-b border-gray-800/50 hover:bg-gray-800/30"
                  >
                    <td className="p-4">
                      <Link
                        to={`/jobs/${job.id}`}
                        className="text-ergon-400 hover:underline font-mono text-xs"
                      >
                        {job.id.slice(0, 8)}...
                      </Link>
                    </td>
                    <td
                      className={`p-4 font-medium capitalize ${statusColor[job.status]}`}
                    >
                      {job.status}
                    </td>
                    <td className="p-4 text-gray-300">{job.environment}</td>
                    <td className="p-4 text-gray-400">
                      {new Date(job.created_at).toLocaleString()}
                    </td>
                    <td className="p-4">
                      <div className="flex items-center gap-2">
                        <div className="w-24 h-1.5 bg-gray-800 rounded-full overflow-hidden">
                          <div
                            className="h-full bg-ergon-500 transition-all"
                            style={{ width: `${(completed / 5) * 100}%` }}
                          />
                        </div>
                        <span className="text-xs text-gray-500">
                          {completed}/5
                        </span>
                      </div>
                    </td>
                    <td className="p-4">
                      {job.status === "completed" && (
                        <Link
                          to={`/reports/${job.id}`}
                          className="text-xs text-ergon-400 hover:underline"
                        >
                          Report
                        </Link>
                      )}
                    </td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        </div>
      </div>
    </AppShell>
  );
}
