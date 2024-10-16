from flask import Flask, Response, render_template, session
import cv2
import subprocess
import time
from flask_socketio import SocketIO, emit
import pty
import os
import select
# make sure to install flask-socketio on jetson b4 running, otherwise itll poop :(
app = Flask(__name__)
app.secret_key = 'colton_is_a_latina_baddie'
socketio = SocketIO(app)
# unrelated but if we wanted 2 be more secure: we could implement a shell sandbox, / display over https / make a better pw than coltin is a latnia baddie
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

def read_and_forward_pty_output(fd):
    max_read_bytes = 1024 * 20
    while True:
        socketio.sleep(0.01)
        if fd in select.select([fd], [], [], 0)[0]:
            output = os.read(fd, max_read_bytes).decode()
            socketio.emit('pty_output', {'output': output}, namespace='/pty')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/terminal')
def terminal():
    return render_template('terminal.html')

@socketio.on('connect', namespace='/pty')
def connect():
    print('Client connected')

    # bash shell
    (child_pid, fd) = pty.fork()
    if child_pid == 0:
        # child process
        os.execvp('bash', ['bash'])
    else:
        # parent process
        socketio.start_background_task(target=read_and_forward_pty_output, fd=fd)
        session['fd'] = fd

@socketio.on('pty_input', namespace='/pty')
def pty_input(data):
    fd = session.get('fd')
    if fd:
        os.write(fd, data['input'].encode())

@socketio.on('disconnect', namespace='/pty')
def disconnect():
    print('Client disconnected')

if __name__ == '__main__':
    socketio.run(app, host='127.0.0.1', port=5000, debug=True)
