import { Footprints } from 'lucide-react'

export default function EmptyState({ icon = <Footprints />, title, subtitle }) {
  return (
    <div className="empty-state">
      <span className="empty-icon">{icon}</span>
      <p className="empty-title">{title}</p>
      {subtitle && <p className="empty-subtitle">{subtitle}</p>}
    </div>
  )
}
