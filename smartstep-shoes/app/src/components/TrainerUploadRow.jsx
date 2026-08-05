import { useState } from 'react'
import { Check, Upload } from 'lucide-react'

export default function TrainerUploadRow({ label, accept, hasFile, onUpload }) {
  const [status, setStatus] = useState('idle') // idle | uploading | done | error

  async function handleFileChange(event) {
    const file = event.target.files[0]
    if (!file) return
    setStatus('uploading')
    try {
      await onUpload(file)
      setStatus('done')
    } catch {
      setStatus('error')
    }
    event.target.value = ''
  }

  return (
    <div className="trainer-upload-row">
      <div className="trainer-upload-info">
        <span className="trainer-upload-label">{label}</span>
        <span className={'trainer-upload-status' + (hasFile ? ' has-file' : '')}>
          {hasFile ? (
            <span style={{ display: 'inline-flex', alignItems: 'center', gap: 4 }}>
              <Check size={13} /> קיים
            </span>
          ) : (
            'לא הועלה'
          )}
        </span>
      </div>
      <label className="trainer-upload-btn">
        <Upload size={14} />
        {status === 'uploading' ? 'מעלה...' : hasFile ? 'החלף קובץ' : 'העלה קובץ'}
        <input type="file" accept={accept} onChange={handleFileChange} disabled={status === 'uploading'} hidden />
      </label>
      {status === 'error' && <span className="trainer-upload-error">שגיאה בהעלאה</span>}
    </div>
  )
}
