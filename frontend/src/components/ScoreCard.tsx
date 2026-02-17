import clsx from "clsx";

interface ScoreCardProps {
  score: number;
  grade?: string;
  label?: string;
  size?: "sm" | "lg";
}

function scoreColor(score: number) {
  if (score >= 80) return "text-green-400 border-green-600";
  if (score >= 60) return "text-yellow-400 border-yellow-600";
  if (score >= 40) return "text-orange-400 border-orange-600";
  return "text-red-400 border-red-600";
}

export default function ScoreCard({
  score,
  grade,
  label = "Overall Score",
  size = "lg",
}: ScoreCardProps) {
  return (
    <div
      className={clsx(
        "bg-gray-900 border-2 rounded-xl text-center",
        scoreColor(score),
        size === "lg" ? "p-8" : "p-4"
      )}
    >
      <div
        className={clsx(
          "font-bold",
          size === "lg" ? "text-5xl" : "text-2xl"
        )}
      >
        {Math.round(score)}
      </div>
      {grade && (
        <div
          className={clsx(
            "text-gray-400 mt-1",
            size === "lg" ? "text-xl" : "text-sm"
          )}
        >
          Grade: {grade}
        </div>
      )}
      <div
        className={clsx(
          "text-gray-500 mt-2",
          size === "lg" ? "text-sm" : "text-xs"
        )}
      >
        {label}
      </div>
    </div>
  );
}
