import clsx from "clsx";
import {
  CheckCircle,
  XCircle,
  Loader2,
  Circle,
} from "lucide-react";
import type { JobStage } from "../lib/api";

const STAGE_LABELS: Record<string, string> = {
  cad_ingestion: "CAD Ingestion",
  digital_twin: "Digital Twin",
  simulation: "Simulation",
  analysis: "Analysis",
  report: "Report Generation",
};

const statusIcons = {
  pending: Circle,
  running: Loader2,
  completed: CheckCircle,
  failed: XCircle,
  skipped: Circle,
};

export default function StageCard({ stage }: { stage: JobStage }) {
  const Icon = statusIcons[stage.status];

  return (
    <div
      className={clsx(
        "flex items-center gap-4 p-4 rounded-lg border",
        stage.status === "completed" && "border-green-800 bg-green-900/10",
        stage.status === "running" && "border-ergon-600 bg-ergon-600/10",
        stage.status === "failed" && "border-red-800 bg-red-900/10",
        stage.status === "pending" && "border-gray-800 bg-gray-900",
        stage.status === "skipped" && "border-gray-800 bg-gray-900 opacity-50"
      )}
    >
      <Icon
        size={20}
        className={clsx(
          stage.status === "completed" && "text-green-400",
          stage.status === "running" && "text-ergon-400 animate-spin",
          stage.status === "failed" && "text-red-400",
          stage.status === "pending" && "text-gray-600",
          stage.status === "skipped" && "text-gray-600"
        )}
      />
      <div className="flex-1">
        <p className="text-sm font-medium text-gray-200">
          {STAGE_LABELS[stage.stage_name] || stage.stage_name}
        </p>
        {stage.message && (
          <p className="text-xs text-gray-500 mt-1">{stage.message}</p>
        )}
        {stage.error_message && (
          <p className="text-xs text-red-400 mt-1">{stage.error_message}</p>
        )}
      </div>
      {stage.status === "running" && (
        <div className="w-24 h-1.5 bg-gray-800 rounded-full overflow-hidden">
          <div
            className="h-full bg-ergon-500 transition-all duration-300"
            style={{ width: `${Math.max(0, stage.progress * 100)}%` }}
          />
        </div>
      )}
    </div>
  );
}
