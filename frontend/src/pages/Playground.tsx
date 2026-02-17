import { useEffect, useRef, useState } from "react";
import { Play, Square, Send } from "lucide-react";
import AppShell from "../components/AppShell";
import { apiGet, startPlayground, stopPlayground, Job } from "../lib/api";
import { usePlayground, ChatMessage } from "../hooks/usePlayground";

const PHASE_COLORS: Record<string, string> = {
  idle: "text-gray-400",
  planning: "text-yellow-400",
  approach: "text-ergon-400",
  descend: "text-ergon-400",
  grasp: "text-orange-400",
  lift: "text-green-400",
  hold: "text-green-400",
  starting: "text-yellow-400",
  stopped: "text-gray-500",
};

export default function Playground() {
  const [jobs, setJobs] = useState<Job[]>([]);
  const [selectedJobId, setSelectedJobId] = useState<string>("");
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [launching, setLaunching] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [commandText, setCommandText] = useState("");
  const chatEndRef = useRef<HTMLDivElement>(null);

  const {
    currentFrame,
    phase,
    statusMessage,
    simStep,
    simTime,
    chatHistory,
    connected,
    sendCommand,
  } = usePlayground(sessionId);

  // Fetch completed jobs
  useEffect(() => {
    apiGet<Job[]>("/jobs")
      .then((all) => setJobs(all.filter((j) => j.status === "completed")))
      .catch(() => {});
  }, []);

  // Auto-scroll chat
  useEffect(() => {
    chatEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [chatHistory]);

  const handleLaunch = async () => {
    if (!selectedJobId) return;
    setLaunching(true);
    setError(null);
    try {
      const res = await startPlayground(selectedJobId);
      setSessionId(res.session_id);
    } catch (e: unknown) {
      setError(e instanceof Error ? e.message : "Failed to start playground");
    } finally {
      setLaunching(false);
    }
  };

  const handleStop = async () => {
    if (!sessionId) return;
    try {
      await stopPlayground(sessionId);
    } catch {
      // best effort
    }
    setSessionId(null);
  };

  const handleSend = () => {
    const text = commandText.trim();
    if (!text) return;
    sendCommand(text);
    setCommandText("");
  };

  return (
    <AppShell>
      <div className="max-w-7xl mx-auto">
        <h1 className="text-3xl font-bold text-white mb-6">Playground</h1>

        {error && (
          <div className="bg-red-900/30 border border-red-800 rounded-lg p-4 text-red-400 mb-6">
            {error}
          </div>
        )}

        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
          {/* Left: Simulation Viewer (2/3) */}
          <div className="lg:col-span-2 flex flex-col gap-4">
            {/* Video frame */}
            <div className="bg-gray-900 border border-gray-800 rounded-lg overflow-hidden">
              <div className="aspect-[4/3] bg-black flex items-center justify-center">
                {currentFrame ? (
                  <img
                    src={`data:image/jpeg;base64,${currentFrame}`}
                    alt="Simulation"
                    className="w-full h-full object-contain"
                  />
                ) : (
                  <span className="text-gray-600 text-sm">
                    {sessionId ? "Waiting for frames..." : "Launch a simulation to begin"}
                  </span>
                )}
              </div>
            </div>

            {/* Status bar */}
            <div className="bg-gray-900 border border-gray-800 rounded-lg px-4 py-3 flex items-center justify-between">
              <div className="flex items-center gap-4 text-sm">
                <span className={PHASE_COLORS[phase] || "text-gray-400"}>
                  {phase.charAt(0).toUpperCase() + phase.slice(1)}
                </span>
                <span className="text-gray-500">
                  Step {simStep}
                </span>
                <span className="text-gray-500">
                  {simTime.toFixed(2)}s
                </span>
                {sessionId && (
                  <span className={connected ? "text-green-500" : "text-red-500"}>
                    {connected ? "Connected" : "Disconnected"}
                  </span>
                )}
              </div>
              {sessionId && (
                <button
                  onClick={handleStop}
                  className="flex items-center gap-2 px-3 py-1.5 bg-red-600/20 text-red-400 rounded-lg hover:bg-red-600/30 transition-colors text-sm font-medium"
                >
                  <Square size={14} />
                  Stop
                </button>
              )}
            </div>
          </div>

          {/* Right: Command Panel (1/3) */}
          <div className="flex flex-col bg-gray-900 border border-gray-800 rounded-lg overflow-hidden h-[calc(100vh-12rem)]">
            {/* Job selector / session info */}
            <div className="p-4 border-b border-gray-800">
              {!sessionId ? (
                <div className="flex flex-col gap-3">
                  <label className="text-sm text-gray-400">Select completed job</label>
                  <select
                    value={selectedJobId}
                    onChange={(e) => setSelectedJobId(e.target.value)}
                    className="w-full bg-gray-800 border border-gray-700 rounded px-3 py-2 text-sm text-gray-200 focus:border-ergon-600 focus:outline-none"
                  >
                    <option value="">Choose a job...</option>
                    {jobs.map((j) => (
                      <option key={j.id} value={j.id}>
                        {j.environment} — {new Date(j.created_at).toLocaleDateString()}
                      </option>
                    ))}
                  </select>
                  <button
                    onClick={handleLaunch}
                    disabled={!selectedJobId || launching}
                    className="flex items-center justify-center gap-2 px-4 py-2 bg-ergon-600 text-white rounded-lg hover:bg-ergon-700 transition-colors text-sm font-medium disabled:opacity-50 disabled:cursor-not-allowed"
                  >
                    <Play size={16} />
                    {launching ? "Launching..." : "Launch"}
                  </button>
                </div>
              ) : (
                <p className="text-sm text-gray-400">
                  Session: <span className="text-gray-200 font-mono">{sessionId}</span>
                </p>
              )}
            </div>

            {/* Chat history */}
            <div className="flex-1 overflow-y-auto p-4 space-y-3">
              {chatHistory.length === 0 && (
                <p className="text-sm text-gray-600 text-center mt-8">
                  {sessionId
                    ? 'Type a command like "pick up the object"'
                    : "Launch a session to get started"}
                </p>
              )}
              {chatHistory.map((msg: ChatMessage) => (
                <div
                  key={msg.id}
                  className={`flex ${msg.role === "user" ? "justify-end" : "justify-start"}`}
                >
                  <div
                    className={`max-w-[85%] rounded-lg px-3 py-2 text-sm ${
                      msg.role === "user"
                        ? "bg-ergon-600/20 text-ergon-300"
                        : "bg-gray-800 text-gray-300"
                    }`}
                  >
                    {msg.text}
                  </div>
                </div>
              ))}
              <div ref={chatEndRef} />
            </div>

            {/* Input */}
            <div className="p-3 border-t border-gray-800">
              <div className="flex gap-2">
                <input
                  type="text"
                  value={commandText}
                  onChange={(e) => setCommandText(e.target.value)}
                  onKeyDown={(e) => e.key === "Enter" && handleSend()}
                  placeholder={sessionId ? "Type a command..." : "Launch session first"}
                  disabled={!sessionId || !connected}
                  className="flex-1 bg-gray-800 border border-gray-700 rounded-lg px-3 py-2 text-sm text-gray-200 placeholder-gray-600 focus:border-ergon-600 focus:outline-none disabled:opacity-50"
                />
                <button
                  onClick={handleSend}
                  disabled={!sessionId || !connected || !commandText.trim()}
                  className="px-3 py-2 bg-ergon-600 text-white rounded-lg hover:bg-ergon-700 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
                >
                  <Send size={16} />
                </button>
              </div>
            </div>
          </div>
        </div>
      </div>
    </AppShell>
  );
}
