import clsx from "clsx";
import { AlertTriangle, AlertCircle, Info } from "lucide-react";

interface Recommendation {
  category: string;
  severity: string;
  title: string;
  description: string;
}

const severityConfig = {
  high: {
    icon: AlertTriangle,
    border: "border-red-800",
    bg: "bg-red-900/10",
    iconColor: "text-red-400",
  },
  medium: {
    icon: AlertCircle,
    border: "border-yellow-800",
    bg: "bg-yellow-900/10",
    iconColor: "text-yellow-400",
  },
  info: {
    icon: Info,
    border: "border-ergon-800",
    bg: "bg-ergon-900/10",
    iconColor: "text-ergon-400",
  },
};

export default function RecommendationList({
  recommendations,
}: {
  recommendations: Recommendation[];
}) {
  return (
    <div className="space-y-3">
      <h3 className="text-sm font-semibold text-gray-300">Recommendations</h3>
      {recommendations.map((rec, i) => {
        const config =
          severityConfig[rec.severity as keyof typeof severityConfig] ||
          severityConfig.info;
        const Icon = config.icon;
        return (
          <div
            key={i}
            className={clsx(
              "flex gap-3 p-4 rounded-lg border",
              config.border,
              config.bg
            )}
          >
            <Icon size={18} className={clsx("mt-0.5 shrink-0", config.iconColor)} />
            <div>
              <p className="text-sm font-medium text-gray-200">{rec.title}</p>
              <p className="text-xs text-gray-400 mt-1">{rec.description}</p>
              <span className="inline-block mt-2 text-xs px-2 py-0.5 rounded bg-gray-800 text-gray-500">
                {rec.category}
              </span>
            </div>
          </div>
        );
      })}
    </div>
  );
}
