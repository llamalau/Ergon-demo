import { useState } from "react";
import { useNavigate } from "react-router-dom";
import AppShell from "../components/AppShell";
import DropZone from "../components/DropZone";
import ModelViewer from "../components/ModelViewer";
import { apiUpload, type UploadRecord } from "../lib/api";

export default function UploadPage() {
  const navigate = useNavigate();
  const [file, setFile] = useState<File | null>(null);
  const [description, setDescription] = useState("");
  const [uploading, setUploading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleUpload = async () => {
    if (!file) return;
    setUploading(true);
    setError(null);
    try {
      const form = new FormData();
      form.append("file", file);
      if (description) form.append("description", description);
      const result = await apiUpload<UploadRecord>("/uploads", form);
      navigate(`/jobs/new?upload_id=${result.id}`);
    } catch (e) {
      setError(e instanceof Error ? e.message : "Upload failed");
    } finally {
      setUploading(false);
    }
  };

  return (
    <AppShell>
      <div className="max-w-3xl mx-auto">
        <h1 className="text-3xl font-bold text-white mb-8">Upload CAD Model</h1>

        <DropZone onFile={setFile} />

        {file && (
          <div className="mt-6 space-y-4">
            <div className="bg-gray-900 rounded-lg p-4 border border-gray-800">
              <p className="text-white font-medium">{file.name}</p>
              <p className="text-gray-500 text-sm">
                {(file.size / 1024 / 1024).toFixed(2)} MB
              </p>
            </div>

            <ModelViewer file={file} />

            <textarea
              value={description}
              onChange={(e) => setDescription(e.target.value)}
              placeholder="Optional description..."
              className="w-full bg-gray-900 border border-gray-800 rounded-lg p-3 text-gray-200 placeholder-gray-600 text-sm resize-none h-20"
            />

            {error && (
              <div className="bg-red-900/30 border border-red-800 rounded-lg p-3 text-red-400 text-sm">
                {error}
              </div>
            )}

            <button
              onClick={handleUpload}
              disabled={uploading}
              className="w-full py-3 bg-ergon-600 text-white rounded-lg hover:bg-ergon-700 transition-colors font-medium disabled:opacity-50 disabled:cursor-not-allowed"
            >
              {uploading ? "Uploading..." : "Upload & Continue"}
            </button>
          </div>
        )}
      </div>
    </AppShell>
  );
}
