import {
  RadarChart,
  PolarGrid,
  PolarAngleAxis,
  PolarRadiusAxis,
  Radar,
  ResponsiveContainer,
} from "recharts";

interface MetricsChartProps {
  subScores: Record<string, number>;
}

export default function MetricsChart({ subScores }: MetricsChartProps) {
  const data = Object.entries(subScores).map(([name, score]) => ({
    metric: name.replace(/_/g, " "),
    score,
    fullMark: 100,
  }));

  return (
    <div className="bg-gray-900 border border-gray-800 rounded-lg p-4">
      <h3 className="text-sm font-semibold text-gray-300 mb-4">
        Quality Breakdown
      </h3>
      <ResponsiveContainer width="100%" height={300}>
        <RadarChart data={data}>
          <PolarGrid stroke="#374151" />
          <PolarAngleAxis
            dataKey="metric"
            tick={{ fill: "#9ca3af", fontSize: 11 }}
          />
          <PolarRadiusAxis
            angle={90}
            domain={[0, 100]}
            tick={{ fill: "#6b7280", fontSize: 10 }}
          />
          <Radar
            name="Score"
            dataKey="score"
            stroke="#0c93e7"
            fill="#0c93e7"
            fillOpacity={0.25}
            strokeWidth={2}
          />
        </RadarChart>
      </ResponsiveContainer>
    </div>
  );
}
