import { NavLink, Route, Routes } from 'react-router-dom'
import { ClipboardList, Footprints, GraduationCap, Home, LineChart, Settings as SettingsIcon, Video } from 'lucide-react'
import Dashboard from './screens/Dashboard.jsx'
import Exercises from './screens/Exercises.jsx'
import History from './screens/History.jsx'
import Charts from './screens/Charts.jsx'
import Settings from './screens/Settings.jsx'
import Trainer from './screens/Trainer.jsx'
import './App.css'

const NAV_ITEMS = [
  { to: '/', label: 'דאשבורד', icon: Home, end: true },
  { to: '/exercises', label: 'תרגילים', icon: Video },
  { to: '/history', label: 'היסטוריה', icon: ClipboardList },
  { to: '/charts', label: 'גרפים', icon: LineChart },
  { to: '/settings', label: 'הגדרות', icon: SettingsIcon },
  { to: '/trainer', label: 'מאמן', icon: GraduationCap },
]

function App() {
  return (
    <div className="app-shell">
      <header className="app-header">
        <span className="app-logo">
          <Footprints size={22} />
        </span>
        <h1>SmartStep</h1>
      </header>

      <nav className="app-nav">
        {NAV_ITEMS.map((item) => (
          <NavLink
            key={item.to}
            to={item.to}
            end={item.end}
            className={({ isActive }) => 'nav-item' + (isActive ? ' active' : '')}
          >
            <span className="nav-icon">
              <item.icon size={20} />
            </span>
            <span className="nav-label">{item.label}</span>
          </NavLink>
        ))}
      </nav>

      <main className="app-main">
        <Routes>
          <Route path="/" element={<Dashboard />} />
          <Route path="/exercises" element={<Exercises />} />
          <Route path="/history" element={<History />} />
          <Route path="/charts" element={<Charts />} />
          <Route path="/settings" element={<Settings />} />
          <Route path="/trainer" element={<Trainer />} />
        </Routes>
      </main>
    </div>
  )
}

export default App
