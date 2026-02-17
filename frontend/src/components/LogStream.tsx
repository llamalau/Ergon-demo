import { useEffect, useRef } from "react";

interface LogEntry {
  stage_name: string;
  message: string;
  timestamp: string;
}

export default function LogStream({ logs }: { logs: LogEntry[] }) {
  const endRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    endRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [logs]);

  return (
    <div className="bg-gray-900 border border-gray-800 rounded-lg p-4 h-64 overflow-y-auto font-mono text-xs">
      {logs.length === 0 && (
        <p className="text-gray-600">Waiting for logs...</p>
      )}
      {logs.map((log, i) => (
        <div key={i} className="flex gap-2 py-0.5">
          <span className="text-gray-600 shrink-0">
            {new Date(log.timestamp).toLocaleTimeString()}
          </span>
          <span className="text-ergon-400 shrink-0">[{log.stage_name}]</span>
          <span className="text-gray-300">{log.message}</span>
        </div>
      ))}
      <div ref={endRef} />
    </div>
  );
}
