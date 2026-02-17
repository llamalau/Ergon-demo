import { useEffect, useState } from "react";
import AppShell from "../components/AppShell";
import { apiGet } from "../lib/api";

interface Environment {
  id: string;
  name: string;
  description: string;
}

export default function Environments() {
  const [envs, setEnvs] = useState<Environment[]>([]);

  useEffect(() => {
    apiGet<Environment[]>("/environments").then(setEnvs).catch(console.error);
  }, []);

  return (
    <AppShell>
      <div className="max-w-4xl mx-auto">
        <h1 className="text-3xl font-bold text-white mb-8">Environments</h1>
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
          {envs.map((env) => (
            <div
              key={env.id}
              className="bg-gray-900 border border-gray-800 rounded-lg p-5 hover:border-gray-700 transition-colors"
            >
              <div className="w-full h-32 bg-gray-800 rounded-lg mb-4 flex items-center justify-center">
                <span className="text-4xl text-gray-600">
                  {env.id === "kitchen" && "🍳"}
                  {env.id === "workshop" && "🔧"}
                  {env.id === "vehicle" && "🚗"}
                  {env.id === "operating_room" && "🏥"}
                  {env.id === "open_space" && "📦"}
                </span>
              </div>
              <h3 className="text-white font-semibold mb-1">{env.name}</h3>
              <p className="text-gray-500 text-sm">{env.description}</p>
            </div>
          ))}
        </div>
      </div>
    </AppShell>
  );
}
