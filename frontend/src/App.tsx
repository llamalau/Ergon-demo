import { Routes, Route, Navigate } from "react-router-dom";
import Landing from "./pages/Landing";
import UploadPage from "./pages/UploadPage";
import Dashboard from "./pages/Dashboard";
import JobDetail from "./pages/JobDetail";
import NewJob from "./pages/NewJob";
import Environments from "./pages/Environments";
import ReportPage from "./pages/ReportPage";
import Playground from "./pages/Playground";

export default function App() {
  return (
    <Routes>
      <Route path="/" element={<Landing />} />
      <Route path="/upload" element={<UploadPage />} />
      <Route path="/dashboard" element={<Dashboard />} />
      <Route path="/jobs" element={<Navigate to="/dashboard" replace />} />
      <Route path="/jobs/new" element={<NewJob />} />
      <Route path="/jobs/:jobId" element={<JobDetail />} />
      <Route path="/environments" element={<Environments />} />
      <Route path="/reports" element={<Navigate to="/dashboard" replace />} />
      <Route path="/reports/:jobId" element={<ReportPage />} />
      <Route path="/playground" element={<Playground />} />
    </Routes>
  );
}
