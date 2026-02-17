import { Suspense } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Stage } from "@react-three/drei";
import * as THREE from "three";

interface HeatmapOverlayProps {
  glbUrl: string | null;
}

function HeatmapModel() {
  // Placeholder — in a full implementation, load the GLB heatmap mesh
  return (
    <mesh>
      <sphereGeometry args={[0.5, 32, 32]} />
      <meshStandardMaterial
        vertexColors
        color="#60a5fa"
        metalness={0.2}
        roughness={0.7}
      />
    </mesh>
  );
}

export default function HeatmapOverlay({ glbUrl }: HeatmapOverlayProps) {
  if (!glbUrl) {
    return (
      <div className="w-full h-64 bg-gray-900 rounded-lg border border-gray-800 flex items-center justify-center text-gray-600 text-sm">
        No heatmap data available
      </div>
    );
  }

  return (
    <div className="bg-gray-900 border border-gray-800 rounded-lg overflow-hidden">
      <div className="px-4 py-2 border-b border-gray-800">
        <h3 className="text-sm font-medium text-gray-300">Contact Heatmap</h3>
      </div>
      <div className="h-64">
        <Canvas camera={{ position: [2, 2, 2], fov: 45 }}>
          <Suspense fallback={null}>
            <Stage environment="city" intensity={0.3}>
              <HeatmapModel />
            </Stage>
          </Suspense>
          <OrbitControls />
        </Canvas>
      </div>
      <div className="px-4 py-2 border-t border-gray-800 flex items-center gap-4 text-xs text-gray-500">
        <span className="flex items-center gap-1">
          <span className="w-3 h-3 rounded-full bg-blue-500" /> Low contact
        </span>
        <span className="flex items-center gap-1">
          <span className="w-3 h-3 rounded-full bg-yellow-500" /> Medium
        </span>
        <span className="flex items-center gap-1">
          <span className="w-3 h-3 rounded-full bg-red-500" /> High contact
        </span>
      </div>
    </div>
  );
}
