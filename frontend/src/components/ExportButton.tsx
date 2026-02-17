import { Download } from "lucide-react";

interface ExportButtonProps {
  jobId: string;
}

export default function ExportButton({ jobId }: ExportButtonProps) {
  return (
    <a
      href={`/api/v1/reports/${jobId}/pdf`}
      download
      className="inline-flex items-center gap-2 px-4 py-2 bg-ergon-600 text-white rounded-lg hover:bg-ergon-700 transition-colors text-sm font-medium"
    >
      <Download size={16} />
      Export PDF
    </a>
  );
}
