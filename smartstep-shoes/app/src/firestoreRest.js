import { FIRESTORE_BASE_URL } from './firebase.js'

function encodeValue(value) {
  if (value === null || value === undefined) return { nullValue: null }
  if (typeof value === 'boolean') return { booleanValue: value }
  if (typeof value === 'number') {
    return Number.isInteger(value) ? { integerValue: String(value) } : { doubleValue: value }
  }
  if (Array.isArray(value)) return { arrayValue: { values: value.map(encodeValue) } }
  if (typeof value === 'object') return { mapValue: { fields: encodeFields(value) } }
  return { stringValue: String(value) }
}

function encodeFields(obj) {
  const fields = {}
  for (const [key, value] of Object.entries(obj)) {
    fields[key] = encodeValue(value)
  }
  return fields
}

function decodeValue(value) {
  if ('nullValue' in value) return null
  if ('booleanValue' in value) return value.booleanValue
  if ('integerValue' in value) return Number(value.integerValue)
  if ('doubleValue' in value) return value.doubleValue
  if ('stringValue' in value) return value.stringValue
  if ('timestampValue' in value) return value.timestampValue
  if ('arrayValue' in value) return (value.arrayValue.values || []).map(decodeValue)
  if ('mapValue' in value) return decodeFields(value.mapValue.fields || {})
  return null
}

function decodeFields(fields) {
  const obj = {}
  for (const [key, value] of Object.entries(fields || {})) {
    obj[key] = decodeValue(value)
  }
  return obj
}

export async function getDocument(path) {
  const res = await fetch(`${FIRESTORE_BASE_URL}/${path}`)
  if (res.status === 404) return null
  if (!res.ok) throw new Error(`Firestore GET ${path} failed: ${res.status}`)
  const json = await res.json()
  return decodeFields(json.fields)
}

export async function listCollection(collectionPath) {
  const res = await fetch(`${FIRESTORE_BASE_URL}/${collectionPath}`)
  if (!res.ok) throw new Error(`Firestore GET ${collectionPath} failed: ${res.status}`)
  const json = await res.json()
  const docs = json.documents || []
  return docs.map((doc) => ({ id: doc.name.split('/').pop(), ...decodeFields(doc.fields) }))
}

export async function setDocument(path, data) {
  const fieldPaths = Object.keys(data).map((key) => `updateMask.fieldPaths=${encodeURIComponent(key)}`).join('&')
  const res = await fetch(`${FIRESTORE_BASE_URL}/${path}?${fieldPaths}`, {
    method: 'PATCH',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ fields: encodeFields(data) }),
  })
  if (!res.ok) throw new Error(`Firestore PATCH ${path} failed: ${res.status}`)
}

export async function addDocument(collectionPath, data) {
  const res = await fetch(`${FIRESTORE_BASE_URL}/${collectionPath}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ fields: encodeFields(data) }),
  })
  if (!res.ok) throw new Error(`Firestore POST ${collectionPath} failed: ${res.status}`)
}
