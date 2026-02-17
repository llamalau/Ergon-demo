import { useState } from "react";

const MATERIALS = [
  "default",
  "plastic_abs",
  "plastic_pla",
  "aluminum",
  "steel",
  "wood",
  "rubber",
  "glass",
  "ceramic",
  "carbon_fiber",
];

interface PropertyOverridesProps {
  onOverrides: (overrides: Record<string, unknown>) => void;
}

export default function PropertyOverrides({
  onOverrides,
}: PropertyOverridesProps) {
  const [material, setMaterial] = useState("default");
  const [mass, setMass] = useState("");
  const [friction, setFriction] = useState("");

  const handleApply = () => {
    const overrides: Record<string, unknown> = { material };
    if (mass) overrides.mass = parseFloat(mass);
    if (friction) overrides.friction = parseFloat(friction);
    onOverrides(overrides);
  };

  return (
    <div className="bg-gray-900 border border-gray-800 rounded-lg p-4 space-y-4">
      <h3 className="text-sm font-semibold text-gray-300">
        Property Overrides (optional)
      </h3>

      <div className="grid grid-cols-3 gap-4">
        <div>
          <label className="block text-xs text-gray-500 mb-1">Material</label>
          <select
            value={material}
            onChange={(e) => setMaterial(e.target.value)}
            className="w-full bg-gray-800 border border-gray-700 rounded px-2 py-1.5 text-sm text-gray-200"
          >
            {MATERIALS.map((m) => (
              <option key={m} value={m}>
                {m.replace("_", " ")}
              </option>
            ))}
          </select>
        </div>

        <div>
          <label className="block text-xs text-gray-500 mb-1">
            Mass (kg, optional)
          </label>
          <input
            type="number"
            step="0.01"
            value={mass}
            onChange={(e) => setMass(e.target.value)}
            placeholder="Auto"
            className="w-full bg-gray-800 border border-gray-700 rounded px-2 py-1.5 text-sm text-gray-200 placeholder-gray-600"
          />
        </div>

        <div>
          <label className="block text-xs text-gray-500 mb-1">
            Friction (0-1, optional)
          </label>
          <input
            type="number"
            step="0.01"
            min="0"
            max="1"
            value={friction}
            onChange={(e) => setFriction(e.target.value)}
            placeholder="Auto"
            className="w-full bg-gray-800 border border-gray-700 rounded px-2 py-1.5 text-sm text-gray-200 placeholder-gray-600"
          />
        </div>
      </div>

      <button
        onClick={handleApply}
        className="text-xs px-3 py-1.5 bg-gray-800 text-gray-300 rounded hover:bg-gray-700 transition-colors"
      >
        Apply Overrides
      </button>
    </div>
  );
}
