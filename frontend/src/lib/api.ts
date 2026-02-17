const API_BASE = "/api/v1";

export async function apiGet<T>(path: string): Promise<T> {
  const res = await fetch(`${API_BASE}${path}`);
  if (!res.ok) {
    const err = await res.json().catch(() => ({ detail: res.statusText }));
    throw new Error(err.detail || res.statusText);
  }
  return res.json();
}

export async function apiPost<T>(
  path: string,
  body?: Record<string, unknown>
): Promise<T> {
  const res = await fetch(`${API_BASE}${path}`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: body ? JSON.stringify(body) : undefined,
  });
  if (!res.ok) {
    const err = await res.json().catch(() => ({ detail: res.statusText }));
    throw new Error(err.detail || res.statusText);
  }
  return res.json();
}

export async function apiUpload<T>(
  path: string,
  formData: FormData
): Promise<T> {
  const res = await fetch(`${API_BASE}${path}`, {
    method: "POST",
    body: formData,
  });
  if (!res.ok) {
    const err = await res.json().catch(() => ({ detail: res.statusText }));
    throw new Error(err.detail || res.statusText);
  }
  return res.json();
}

export interface UploadRecord {
  id: string;
  filename: string;
  original_filename: string;
  content_type: string;
  file_size: number;
  file_format: string;
  description: string | null;
  created_at: string;
}

export interface JobStage {
  id: string;
  stage_name: string;
  stage_order: number;
  status: "pending" | "running" | "completed" | "failed" | "skipped";
  progress: number;
  message: string | null;
  result_data: Record<string, unknown> | null;
  error_message: string | null;
  started_at: string | null;
  completed_at: string | null;
}

export interface Job {
  id: string;
  upload_id: string;
  status: "pending" | "running" | "completed" | "failed";
  environment: string;
  config: Record<string, unknown> | null;
  error_message: string | null;
  celery_task_id: string | null;
  created_at: string;
  started_at: string | null;
  completed_at: string | null;
  stages: JobStage[];
}

export interface VideoInfo {
  task_type: string;
  trial_index: number;
  filename: string;
  video_url: string;
}

export async function getJobVideos(jobId: string): Promise<VideoInfo[]> {
  return apiGet<VideoInfo[]>(`/jobs/${jobId}/videos`);
}

export interface Report {
  id: string;
  job_id: string;
  overall_score: number;
  sub_scores: Record<string, number>;
  metrics: Record<string, unknown>;
  recommendations: Array<Record<string, unknown>>;
  pdf_storage_key: string | null;
  created_at: string;
}
