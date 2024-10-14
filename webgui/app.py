from flask import Flask, render_template, Response, jsonify
import cv2

app = Flask(__name__)

# Initialize cameras (replace 0 and 1 with your actual camera indices or streaming endpoints)
lidar_camera = cv2.VideoCapture(0)  # LiDAR Depth Camera
rgb_camera = cv2.VideoCapture(1)    # RGB Camera

def generate_frames(camera):
    """Yield frames from the specified camera in a format suitable for HTML video streaming."""
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    """Render the main control interface with video feed and options."""
    return render_template('index.html')

@app.route('/video_feed/<cam_type>')
def video_feed(cam_type):
    """Serve video feed based on camera type (lidar or rgb)."""
    if cam_type == 'lidar':
        return Response(generate_frames(lidar_camera), mimetype='multipart/x-mixed-replace; boundary=frame')
    elif cam_type == 'rgb':
        return Response(generate_frames(rgb_camera), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/set_mode/<mode>', methods=['POST'])
def set_mode(mode):
    """Handle robot mode switching."""
    # Implement robot control logic here for Autonomous 1, Autonomous 2, or Manual modes
    print(f"Mode set to: {mode}")
    return jsonify({'status': 'success', 'mode': mode})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
