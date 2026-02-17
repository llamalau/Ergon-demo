import { Suspense, useMemo } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Stage } from "@react-three/drei";
import * as THREE from "three";

interface ModelViewerProps {
  file: File | null;
  url?: string;
}

function STLModel({ geometry }: { geometry: THREE.BufferGeometry }) {
  return (
    <mesh geometry={geometry}>
      <meshStandardMaterial color="#60a5fa" metalness={0.3} roughness={0.6} />
    </mesh>
  );
}

function ModelScene({ file }: { file: File }) {
  const geometry = useMemo(() => {
    // For now, show a placeholder box. Full STL parsing will be loaded async.
    const geo = new THREE.BoxGeometry(1, 1, 1);
    return geo;
  }, [file]);

  return <STLModel geometry={geometry} />;
}

export default function ModelViewer({ file }: ModelViewerProps) {
  if (!file) {
    return (
      <div className="w-full h-80 bg-gray-900 rounded-xl border border-gray-800 flex items-center justify-center text-gray-600">
        No model loaded
      </div>
    );
  }

  return (
    <div className="w-full h-80 bg-gray-900 rounded-xl border border-gray-800 overflow-hidden">
      <Canvas camera={{ position: [3, 3, 3], fov: 45 }}>
        <Suspense fallback={null}>
          <Stage environment="city" intensity={0.5}>
            <ModelScene file={file} />
          </Stage>
        </Suspense>
        <OrbitControls />
      </Canvas>
    </div>
  );
}
