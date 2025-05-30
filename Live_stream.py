# ========== IMPORTS ==========
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2
import cv2
import time

# ========== FLASK APP INITIALIZATION ==========
app = Flask(__name__)

# ========== CAMERA CONFIGURATION ==========
picam2 = Picamera2()  # Initialize Picamera2 object

# Find sensor resolution (e.g., 2592x1944 for HQ Cam)
sensor_size = picam2.sensor_resolution
print("Sensor size:", sensor_size)

# Create video configuration with:
# - Main stream: 1280x720 resolution, RGB888 format
# - Frame rate control: ~60fps (16666µs frame duration)
camera_config = picam2.create_video_configuration(
    main={"size": (1280, 720), "format": "RGB888"},
    controls={"FrameDurationLimits": (16666, 16666)}
)

picam2.configure(camera_config)  # Apply camera configuration
picam2.start()  # Start camera

# Important: Wait briefly then set ScalerCrop directly
time.sleep(1)
picam2.set_controls({"ScalerCrop": (0, 0, sensor_size[0], sensor_size[1])})

# ========== HTML/CSS TEMPLATE ==========
html_page = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Rover Live Stream</title>
    <style>
        /* Dark theme styling */
        body {
            background-color: #121212;
            color: #eee;
            font-family: 'Segoe UI', sans-serif;
            display: flex;
            flex-direction: column;
            align-items: center;
            padding-top: 30px;
            margin: 0;
        }
        /* Header styling with cyan color */
        h1 {
            margin-bottom: 20px;
            font-size: 2.5rem;
            color: #00ffe7;
        }
        /* Video container with glow effect */
        .video-container {
            background: #222;
            padding: 12px;
            border-radius: 12px;
            box-shadow: 0 0 20px rgba(0,255,255,0.3);
        }
        /* Image styling with responsive sizing */
        img {
            border-radius: 8px;
            max-width: 90vw;
            max-height: 80vh;
            box-shadow: 0 0 10px rgba(0,255,255,0.5);
        }
        /* Footer styling */
        footer {
            margin-top: 20px;
            font-size: 0.9rem;
            color: #888;
        }
    </style>
</head>
<body>
    <h1>🔴 Rover Live Stream</h1>
    <div class="video-container">
        <img src="{{ url_for('video_feed') }}" alt="Live Stream">
    </div>
    <footer>
        Pi Rover Cam &copy; 2025
    </footer>
</body>
</html>
"""

# ========== ROUTES ==========
@app.route('/')
def index():
    return render_template_string(html_page)  # Serve the HTML page

@app.route('/video_feed')
def video_feed():
    # Stream video with multipart/x-mixed-replace content type
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# ========== VIDEO FRAME GENERATION ==========
def generate_frames():
    while True:
        frame = picam2.capture_array()  # Capture frame from camera
        # Encode frame as JPEG with 70% quality
        ret, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not ret:
            continue  # Skip if encoding failed
        # Yield frame in multipart format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

# ========== MAIN EXECUTION ==========
if __name__ == '__main__':
    # Run Flask app on all interfaces, port 5000, with threading
    app.run(host='0.0.0.0', port=5000, threaded=True)
