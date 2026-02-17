import { useEffect, useRef, useState, useCallback } from "react";

export interface ChatMessage {
  id: string;
  role: "user" | "system";
  text: string;
  timestamp: number;
}

interface PlaygroundState {
  currentFrame: string | null;
  phase: string;
  statusMessage: string;
  simStep: number;
  simTime: number;
  chatHistory: ChatMessage[];
  connected: boolean;
  sendCommand: (text: string) => void;
}

export function usePlayground(sessionId: string | null): PlaygroundState {
  const [currentFrame, setCurrentFrame] = useState<string | null>(null);
  const [phase, setPhase] = useState("idle");
  const [statusMessage, setStatusMessage] = useState("");
  const [simStep, setSimStep] = useState(0);
  const [simTime, setSimTime] = useState(0);
  const [chatHistory, setChatHistory] = useState<ChatMessage[]>([]);
  const [connected, setConnected] = useState(false);
  const wsRef = useRef<WebSocket | null>(null);
  const msgIdRef = useRef(0);

  useEffect(() => {
    if (!sessionId) {
      setConnected(false);
      return;
    }

    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const ws = new WebSocket(
      `${protocol}//${window.location.host}/api/v1/playground/ws/${sessionId}`
    );
    wsRef.current = ws;

    ws.onopen = () => setConnected(true);
    ws.onclose = () => setConnected(false);

    ws.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);

        if (msg.type === "frame") {
          setCurrentFrame(msg.data);
          setSimStep(msg.step);
          setSimTime(msg.sim_time);
          if (msg.phase) setPhase(msg.phase);
        } else if (msg.type === "status") {
          setPhase(msg.phase);
          setStatusMessage(msg.message);
          setSimStep(msg.step);
          setSimTime(msg.sim_time);
          setChatHistory((prev) => [
            ...prev,
            {
              id: `sys-${++msgIdRef.current}`,
              role: "system",
              text: msg.message,
              timestamp: Date.now(),
            },
          ]);
        }
        // command_ack is intentionally ignored
      } catch {
        // ignore parse errors
      }
    };

    return () => {
      ws.close();
      wsRef.current = null;
    };
  }, [sessionId]);

  const sendCommand = useCallback(
    (text: string) => {
      if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) return;
      wsRef.current.send(JSON.stringify({ type: "command", text }));
      setChatHistory((prev) => [
        ...prev,
        {
          id: `usr-${++msgIdRef.current}`,
          role: "user",
          text,
          timestamp: Date.now(),
        },
      ]);
    },
    []
  );

  return {
    currentFrame,
    phase,
    statusMessage,
    simStep,
    simTime,
    chatHistory,
    connected,
    sendCommand,
  };
}
