const PROJECT_ID = import.meta.env.VITE_FIREBASE_PROJECT_ID;

export const FIRESTORE_BASE_URL = `https://firestore.googleapis.com/v1/projects/${PROJECT_ID}/databases/(default)/documents`;
