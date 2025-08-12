# scripts/web_dashboard.py
import time, cv2
from flask import Flask, Response, jsonify, render_template_string
from threading import Thread, Lock
from collections import Counter, deque

def start_web_dashboard(classifier, host="0.0.0.0", port=8000, infer_hz=5.0):
    """
    Simple Flask dashboard:
      /            -> HTML page
      /stats       -> JSON {label, confidence, fps, counts}
      /video.mjpg  -> MJPEG stream with overlay
    'classifier' must implement:
      - capture_frame() -> RGB ndarray
      - preprocess_frame(frame_rgb) -> tensor
      - run_inference(tensor) -> (idx, conf, probs)
      - create_overlay_text(frame_rgb, results) -> BGR ndarray with overlay
      - class_names[list]
    """
    app = Flask(__name__)
    lock = Lock()
    latest_jpeg = None
    last_label, last_conf = "", 0.0
    counts = Counter()
    frame_times = deque(maxlen=60)
    boundary = "frame"
    running = True

    def worker():
        nonlocal latest_jpeg, last_label, last_conf, running
        interval = 1.0 / max(0.1, infer_hz)
        next_t = time.time()
        while running:
            now = time.time()
            if now < next_t:
                time.sleep(min(0.005, next_t - now))
                continue
            next_t = now + interval

            # Capture
            frame = classifier.capture_frame()  # RGB
            with lock:
                frame_times.append(time.time())

            # Inference
            x = classifier.preprocess_frame(frame)
            idx, conf, probs = classifier.run_inference(x)
            label = classifier.class_names[idx] if idx < len(classifier.class_names) else f"Unknown_{idx}"

            results = {
                "predicted_class": label,
                "class_index": idx,
                "confidence": conf,
                "frame": frame,
                "all_probabilities": {
                    classifier.class_names[i]: float(probs[i])
                    for i in range(min(len(probs), len(classifier.class_names)))
                }
            }

            # Overlay -> JPEG
            disp = classifier.create_overlay_text(frame, results)
            ok, buf = cv2.imencode(".jpg", disp, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if not ok:
                continue

            with lock:
                latest_jpeg = buf.tobytes()
                last_label = label
                last_conf = float(conf)
                counts[label] += 1

    @app.route("/")
    def index():
        html = """
        <!doctype html><html><head>
          <meta charset="utf-8"/><meta name="viewport" content="width=device-width, initial-scale=1">
          <title>Recycling Robot – Dashboard</title>
          <style>
            body { font: 16px system-ui, sans-serif; margin: 20px; color: #111; }
            .wrap { display: grid; gap: 16px; grid-template-columns: 1fr 320px; align-items: start; }
            img { width: 100%; max-width: 100%; border-radius: 12px; }
            .card { padding: 12px 14px; border: 1px solid #e5e7eb; border-radius: 12px; box-shadow: 0 1px 4px rgba(0,0,0,.05); }
            .k { color: #6b7280; }
            .stat { display: flex; justify-content: space-between; padding: 6px 0; border-bottom: 1px dashed #e5e7eb; }
            .stat:last-child { border: none; }
            pre { margin: 0; white-space: pre-wrap; }
            button { padding: 8px 12px; border-radius: 10px; border: 1px solid #d1d5db; background: #fff; cursor: pointer; }
          </style>
        </head><body>
          <h1>Recycling Robot · Live</h1>
          <div class="wrap">
            <div class="card"><img id="video" src="/video.mjpg" alt="Live video"/></div>
            <div class="card">
              <div class="stat"><span class="k">Label</span><strong id="label">—</strong></div>
              <div class="stat"><span class="k">Confidence</span><strong id="conf">—</strong></div>
              <div class="stat"><span class="k">FPS</span><strong id="fps">—</strong></div>
              <div class="stat"><span class="k">Counts</span><pre id="counts"></pre></div>
              <div style="margin-top:10px;"><button onclick="loadStats()">Refresh stats</button></div>
            </div>
          </div>
        <script>
        async function loadStats(){
          try{
            const r = await fetch('/stats'); const s = await r.json();
            document.getElementById('label').textContent = s.label || '—';
            document.getElementById('conf').textContent  = (s.confidence||0).toFixed(2);
            document.getElementById('fps').textContent   = (s.fps||0).toFixed(1);
            document.getElementById('counts').textContent = JSON.stringify(s.counts||{}, null, 2);
          }catch(e){}
        }
        setInterval(loadStats, 1000); loadStats();
        </script>
        </body></html>
        """
        return render_template_string(html)

    @app.route("/stats")
    def stats():
        with lock:
            fps = 0.0
            if len(frame_times) >= 2:
                dt = frame_times[-1] - frame_times[0]
                if dt > 0: fps = (len(frame_times)-1)/dt
            return jsonify({
                "label": last_label,
                "confidence": last_conf,
                "fps": round(fps, 1),
                "counts": dict(counts),
            })

    @app.route("/video.mjpg")
    def video():
        def gen():
            while True:
                with lock:
                    jpeg = latest_jpeg
                if jpeg is None:
                    time.sleep(0.01); continue
                yield (b"--" + boundary.encode() + b"\r\n"
                       b"Content-Type: image/jpeg\r\n"
                       b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n" +
                       jpeg + b"\r\n")
        return Response(gen(), mimetype=f"multipart/x-mixed-replace; boundary="+boundary)

    t = Thread(target=worker, daemon=True); t.start()
    try:
        app.run(host=host, port=port, threaded=True)
    finally:
        pass
