import { useEffect, useState } from "react";
import { useSearchParams, useNavigate } from "react-router-dom";
import AppShell from "../components/AppShell";
import PropertyOverrides from "../components/PropertyOverrides";
import { apiGet, apiPost, type Job } from "../lib/api";

interface Environment {
  id: string;
  name: string;
  description: string;
}

export default function NewJob() {
  const [searchParams] = useSearchParams();
  const navigate = useNavigate();
  const uploadId = searchParams.get("upload_id");

  const [envs, setEnvs] = useState<Environment[]>([]);
  const [environment, setEnvironment] = useState("open_space");
  const [overrides, setOverrides] = useState<Record<string, unknown>>({});
  const [submitting, setSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    apiGet<Environment[]>("/environments").then(setEnvs).catch(console.error);
  }, []);

  const handleSubmit = async () => {
    if (!uploadId) return;
    setSubmitting(true);
    setError(null);
    try {
      const job = await apiPost<Job>("/jobs", {
        upload_id: uploadId,
        environment,
        config: { property_overrides: overrides },
      });
      navigate(`/jobs/${job.id}`);
    } catch (e) {
      setError(e instanceof Error ? e.message : "Failed to create job");
    } finally {
      setSubmitting(false);
    }
  };

  if (!uploadId) {
    return (
      <AppShell>
        <div className="max-w-3xl mx-auto text-gray-500">
          No upload ID provided. Please upload a file first.
        </div>
      </AppShell>
    );
  }

  return (
    <AppShell>
      <div className="max-w-3xl mx-auto">
        <h1 className="text-3xl font-bold text-white mb-8">
          Configure Job
        </h1>

        <div className="space-y-6">
          <div className="bg-gray-900 border border-gray-800 rounded-lg p-4">
            <label className="block text-sm text-gray-400 mb-2">
              Environment
            </label>
            <div className="grid grid-cols-2 md:grid-cols-3 gap-3">
              {envs.map((env) => (
                <button
                  key={env.id}
                  onClick={() => setEnvironment(env.id)}
                  className={`p-3 rounded-lg border text-left text-sm transition-colors ${
                    environment === env.id
                      ? "border-ergon-600 bg-ergon-600/10 text-white"
                      : "border-gray-800 bg-gray-800/50 text-gray-400 hover:border-gray-700"
                  }`}
                >
                  <p className="font-medium">{env.name}</p>
                  <p className="text-xs mt-1 opacity-70">{env.description}</p>
                </button>
              ))}
            </div>
          </div>

          <PropertyOverrides onOverrides={setOverrides} />

          {error && (
            <div className="bg-red-900/30 border border-red-800 rounded-lg p-3 text-red-400 text-sm">
              {error}
            </div>
          )}

          <button
            onClick={handleSubmit}
            disabled={submitting}
            className="w-full py-3 bg-ergon-600 text-white rounded-lg hover:bg-ergon-700 transition-colors font-medium disabled:opacity-50"
          >
            {submitting ? "Starting Pipeline..." : "Start Pipeline"}
          </button>
        </div>
      </div>
    </AppShell>
  );
}
