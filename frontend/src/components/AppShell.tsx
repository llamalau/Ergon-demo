import { Link, useLocation } from "react-router-dom";
import {
  Upload,
  LayoutDashboard,
  Play,
  FileText,
  Box,
} from "lucide-react";
import clsx from "clsx";

const navItems = [
  { to: "/dashboard", label: "Dashboard", icon: LayoutDashboard },
  { to: "/upload", label: "Upload", icon: Upload },
  { to: "/jobs", label: "Jobs", icon: Play },
  { to: "/environments", label: "Environments", icon: Box },
  { to: "/reports", label: "Reports", icon: FileText },
];

export default function AppShell({ children }: { children: React.ReactNode }) {
  const location = useLocation();

  return (
    <div className="flex h-screen bg-gray-950">
      {/* Sidebar */}
      <aside className="w-64 bg-gray-900 border-r border-gray-800 flex flex-col">
        <Link to="/" className="p-6 text-2xl font-bold text-white">
          Ergon
        </Link>
        <nav className="flex-1 px-3 space-y-1">
          {navItems.map(({ to, label, icon: Icon }) => (
            <Link
              key={to}
              to={to}
              className={clsx(
                "flex items-center gap-3 px-3 py-2 rounded-lg text-sm font-medium transition-colors",
                location.pathname.startsWith(to)
                  ? "bg-ergon-600/20 text-ergon-400"
                  : "text-gray-400 hover:text-gray-200 hover:bg-gray-800"
              )}
            >
              <Icon size={18} />
              {label}
            </Link>
          ))}
        </nav>
        <div className="p-4 text-xs text-gray-600">
          CAD to Manipulation Quality
        </div>
      </aside>

      {/* Main content */}
      <main className="flex-1 overflow-auto">
        <div className="p-8">{children}</div>
      </main>
    </div>
  );
}
