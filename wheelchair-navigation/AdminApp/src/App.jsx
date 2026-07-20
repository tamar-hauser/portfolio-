import { useState } from 'react'
import { BrowserRouter, Routes, Route, Navigate } from 'react-router-dom'
import Login from './pages/Login.jsx'
import Dashboard from './pages/Dashboard.jsx'
import FrameDetail from './pages/FrameDetail.jsx'

export default function App() {
  const [user, setUser] = useState(null)

  return (
    <BrowserRouter>
      <Routes>
        <Route path="/login" element={
          user ? <Navigate to="/" replace /> : <Login onLogin={setUser} />
        } />
        <Route path="/" element={
          user ? <Dashboard user={user} onLogout={() => setUser(null)} />
               : <Navigate to="/login" replace />
        } />
        <Route path="/frame/:bucket" element={
          user ? <FrameDetail /> : <Navigate to="/login" replace />
        } />
        <Route path="*" element={<Navigate to="/" replace />} />
      </Routes>
    </BrowserRouter>
  )
}
