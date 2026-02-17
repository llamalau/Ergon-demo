import { useCallback, useState } from "react";
import { Upload } from "lucide-react";
import clsx from "clsx";

interface DropZoneProps {
  onFile: (file: File) => void;
  accept?: string;
}

export default function DropZone({
  onFile,
  accept = ".stl,.obj,.step,.stp,.iges,.igs,.urdf,.xml,.mjcf",
}: DropZoneProps) {
  const [dragOver, setDragOver] = useState(false);

  const handleDrop = useCallback(
    (e: React.DragEvent) => {
      e.preventDefault();
      setDragOver(false);
      const file = e.dataTransfer.files[0];
      if (file) onFile(file);
    },
    [onFile]
  );

  const handleChange = useCallback(
    (e: React.ChangeEvent<HTMLInputElement>) => {
      const file = e.target.files?.[0];
      if (file) onFile(file);
    },
    [onFile]
  );

  return (
    <label
      onDragOver={(e) => {
        e.preventDefault();
        setDragOver(true);
      }}
      onDragLeave={() => setDragOver(false)}
      onDrop={handleDrop}
      className={clsx(
        "flex flex-col items-center justify-center w-full h-64 border-2 border-dashed rounded-xl cursor-pointer transition-colors",
        dragOver
          ? "border-ergon-400 bg-ergon-600/10"
          : "border-gray-700 bg-gray-900 hover:border-gray-600"
      )}
    >
      <Upload
        size={40}
        className={clsx(
          "mb-4",
          dragOver ? "text-ergon-400" : "text-gray-500"
        )}
      />
      <p className="text-gray-400 text-sm mb-1">
        Drag and drop a CAD file here, or click to browse
      </p>
      <p className="text-gray-600 text-xs">
        Supports STL, OBJ, STEP, IGES, URDF, MJCF
      </p>
      <input
        type="file"
        accept={accept}
        onChange={handleChange}
        className="hidden"
      />
    </label>
  );
}
