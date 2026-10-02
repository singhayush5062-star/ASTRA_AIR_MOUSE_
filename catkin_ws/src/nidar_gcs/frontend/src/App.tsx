import { Routes, Route, Navigate } from 'react-router-dom';
import ModeSelection from '@/pages/ModeSelection';
import SimulationDashboard from '@/pages/Simulation';
import HardwareDashboard from '@/pages/Hardware';

export default function App() {
  return (
    <Routes>
      <Route path="/"           element={<ModeSelection />} />
      <Route path="/simulation" element={<SimulationDashboard />} />
      <Route path="/hardware"   element={<HardwareDashboard />} />
      <Route path="*"           element={<Navigate to="/" replace />} />
    </Routes>
  );
}
