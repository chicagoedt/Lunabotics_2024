from flask import Flask, Response, render_template, request
import cv2
import subprocess
import time
from flask_socketio import SocketIO, emit, disconnect
import pty
import os
import select

app = Flask(__name__)
app.secret_key = 'secret!'
socketio = SocketIO(app)

# stupid dictinary fix pls
fd_dict = {}

def generate_frames():

    cap = cv2.VideoCapture(0)

    while True:

        success, frame = cap.read()
        if not success:
            break
        

        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        
        frame_bytes = buffer.tobytes()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    cap.release()

def read_and_forward_pty_output(sid, fd):
    max_read_bytes = 1024 * 20
    while True:
        socketio.sleep(0.01)
        try:
            if fd in select.select([fd], [], [], 0)[0]:
                output = os.read(fd, max_read_bytes).decode()
                socketio.emit('pty_output', {'output': output}, namespace='/pty', to=sid)
        except OSError:
            # file descriptor might have been closed
            break

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on('connect', namespace='/pty')
def connect():
    print('Client connected:', request.sid)

    # bash shell
    (child_pid, fd) = pty.fork()
    if child_pid == 0:
        # child process
        os.execvp('bash', ['bash'])
    else:
        # parent process
        fd_dict[request.sid] = fd
        socketio.start_background_task(target=read_and_forward_pty_output, sid=request.sid, fd=fd)

@socketio.on('pty_input', namespace='/pty')
def pty_input(data):
    fd = fd_dict.get(request.sid)
    if fd:
        try:
            os.write(fd, data['input'].encode())
        except OSError:
            # handle the error if fd is closed
            pass
    else:
        print('No fd found for session:', request.sid)

@socketio.on('disconnect', namespace='/pty')
def disconnect():
    print('Client disconnected:', request.sid)
    fd = fd_dict.get(request.sid)
    if fd:
        os.close(fd)
        del fd_dict[request.sid]

if __name__ == '__main__':
    socketio.run(app, host='192.168.0.0', port=5000, debug=True)
