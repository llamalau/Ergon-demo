import { useEffect, useRef, useState } from "react";

interface ProgressEvent {
  job_id: string;
  stage_name: string;
  stage_order: number;
  progress: number;
  message: string;
  timestamp: string;
}

export function useJobProgress(jobId: string | null) {
  const [events, setEvents] = useState<ProgressEvent[]>([]);
  const wsRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    if (!jobId) return;

    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const ws = new WebSocket(
      `${protocol}//${window.location.host}/api/v1/jobs/ws/${jobId}/progress`
    );
    wsRef.current = ws;

    ws.onmessage = (event) => {
      try {
        const data: ProgressEvent = JSON.parse(event.data);
        setEvents((prev) => [...prev, data]);
      } catch {
        // ignore parse errors
      }
    };

    return () => {
      ws.close();
      wsRef.current = null;
    };
  }, [jobId]);

  return events;
}
