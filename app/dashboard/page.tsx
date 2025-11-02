"use client"

import { useEffect, useState } from "react"
import { getFirestore, collection, query, orderBy, onSnapshot } from "firebase/firestore"
import { initializeApp } from "firebase/app"

const firebaseConfig = {
  apiKey: "AIzaSyBpnRpno94ENJtG0x8V8yt5CuYes9PBPbI",
  authDomain: "phoneshop-5866b.firebaseapp.com",
  projectId: "phoneshop-5866b",
  storageBucket: "phoneshop-5866b.appspot.com",
  messagingSenderId: "1087604929685",
  appId: "1:1087604929685:web:85b68a15c9b62f8fbc179c",
  measurementId: "G-8857TZXV31"
}

const app = initializeApp(firebaseConfig)
const db = getFirestore(app)

export default function Dashboard() {
  const [events, setEvents] = useState([])
  const [inferences, setInferences] = useState([])

  useEffect(() => {
    const unsubEvents = onSnapshot(
      query(collection(db, "events"), orderBy("Timestamp", "desc")),
      (snapshot) => {
        setEvents(snapshot.docs.map(doc => ({ id: doc.id, ...doc.data() })))
      }
    )
    const unsubInferences = onSnapshot(
      query(collection(db, "inferences"), orderBy("stats.infer_ms", "desc")),
      (snapshot) => {
        setInferences(snapshot.docs.map(doc => ({ id: doc.id, ...doc.data() })))
      }
    )
    return () => { unsubEvents(); unsubInferences(); }
  }, [])

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 to-slate-800 p-6">
      <div className="max-w-6xl mx-auto">
        <h1 className="text-4xl font-bold text-white mb-8">Dashboard ADAS Realtime</h1>
        <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
          <div>
            <h2 className="text-2xl text-white mb-4">Sự kiện cảnh báo</h2>
            <div className="bg-slate-800 rounded-lg p-4 max-h-[400px] overflow-y-auto">
              {events.map(ev => (
                <div key={ev.id} className="border-b border-slate-700 py-2 text-white">
                  <div className="font-semibold">{ev.EventType}</div>
                  <div>{ev.Description}</div>
                  <div className="text-xs text-slate-400">{new Date(ev.Timestamp).toLocaleString()}</div>
                </div>
              ))}
            </div>
          </div>
          <div>
            <h2 className="text-2xl text-white mb-4">Inference gần nhất</h2>
            <div className="bg-slate-800 rounded-lg p-4 max-h-[400px] overflow-y-auto">
              {inferences.map(inf => (
                <div key={inf.id} className="border-b border-slate-700 py-2 text-white">
                  <div className="font-semibold">Model: {inf.stats?.model || "unknown"}</div>
                  <div>Detections: {inf.detections?.length || 0}</div>
                  <div>Infer time: {inf.stats?.infer_ms} ms</div>
                  <div className="text-xs text-slate-400">ID: {inf.id}</div>
                </div>
              ))}
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}
