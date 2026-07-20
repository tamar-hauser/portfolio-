// Renders a labelled horizontal bar for a 0-1 confidence value
export default function ConfidenceBar({ value, showLabel = true }) {
  const pct = value != null ? Math.max(0, Math.min(1, value)) * 100 : null

  const color =
    pct == null   ? '#CBE0F0'  :
    pct >= 70     ? '#2EB89A'  :
    pct >= 40     ? '#D97706'  : '#D94F4F'

  const bg =
    pct == null   ? '#F0F6FB'  :
    pct >= 70     ? '#E6F7F4'  :
    pct >= 40     ? '#FFFBEB'  : '#FEF2F2'

  return (
    <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
      <div style={{
        flex: 1,
        height: 6,
        background: '#EFF6FC',
        borderRadius: 3,
        overflow: 'hidden',
      }}>
        <div style={{
          height: '100%',
          width: pct != null ? `${pct}%` : '0%',
          background: color,
          borderRadius: 3,
          transition: 'width 0.4s ease',
        }} />
      </div>
      {showLabel && (
        <span style={{
          fontSize: 11,
          fontWeight: 700,
          color,
          background: bg,
          padding: '1px 7px',
          borderRadius: 10,
          minWidth: 38,
          textAlign: 'center',
        }}>
          {pct != null ? `${pct.toFixed(0)}%` : '—'}
        </span>
      )}
    </div>
  )
}
