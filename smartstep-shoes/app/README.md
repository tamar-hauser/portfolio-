# SmartStep Dashboard

React + Vite web app for the SmartStep Shoes system. Reads sessions written by
the coach device to Firebase and presents them to the trainee and the trainer.

## Screens

| Screen | Purpose |
|---|---|
| `Dashboard` | Live session state — current exercise, score, rep count, coach status. |
| `Charts` | Score and pressure trends across sessions. |
| `History` | Past sessions with per-set detail. |
| `Exercises` | Exercise library with reference videos. |
| `Trainer` | Upload a reference video for an exercise. |
| `Settings` | Thresholds and device preferences. |

`FootPressureMap` renders the FSR readings as a foot heat map; `utils/liveSim.js`
feeds simulated data so the UI can be developed without hardware attached.

## Setup

```bash
npm install
cp .env.example .env    # fill in your own Firebase project keys
npm run dev
```

`.env` is untracked — it holds real Firebase credentials. `.env.example` lists
the variables you need to supply.
