import type { JobStage } from "../lib/api";
import StageCard from "./StageCard";

export default function PipelineTracker({ stages }: { stages: JobStage[] }) {
  const sorted = [...stages].sort((a, b) => a.stage_order - b.stage_order);

  return (
    <div className="space-y-3">
      <h2 className="text-lg font-semibold text-white mb-4">Pipeline Progress</h2>
      {sorted.map((stage) => (
        <StageCard key={stage.id} stage={stage} />
      ))}
    </div>
  );
}
