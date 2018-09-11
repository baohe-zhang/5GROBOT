from flask import Flask, render_template, redirect, url_for
import json
import requests

app = Flask(__name__)

commnad = 'STOP'
robot = 'robot1'
speed = 15
angle = 0
track = 0
detect = 0

robot_url = {
	'robot1': 'http://172.24.1.5:8080/restconf/config/robotApp/',
	'robot2': 'http://172.24.1.6:8080/restconf/config/robotApp/',
	'robot3': 'http://172.24.1.7:8080/restconf/config/robotApp/',
	'robot4': 'http://172.24.1.8:8080/restconf/config/robotApp/',
	'robot5': 'http://172.24.1.9:8080/restconf/config/robotApp/'
}       

def sendCommand():
	msg = {
		"robotId": robot,
		"command": command,
		"speed": speed,
		"angle": angle,
		"TrackStatus": track
	}
	print(json.dumps(msg))

	# send HTTP POST to robot's server
	try:
		res = requests.put(robot_url[robot], json=msg, timeout=0.1)
	except Exception:
		print('Connection to Robot Failed.')
	else:
		print('Command Send Successfully.')

@app.route('/log')
def log():
	msg = {
		"robotId": robot,
		"command": command,
		"speed": speed,
		"angle": angle,
		"TrackStatus": track
	}
	return json.dumps(msg)

@app.route('/')
def index():
	return render_template('control.html', robot=robot, speed=speed, 
		angle=angle, track=track, detect=detect)

@app.route('/forward')
def forward():
	global command
	command = 'FORWARD'
	sendCommand()
	log()
	return redirect(url_for('index'))

@app.route('/left')
def left():
	global command
	command = 'LEFT'
	sendCommand()
	log()
	return redirect(url_for('index'))

@app.route('/stop')
def stop():
	global command
	command = 'STOP'
	sendCommand()
	return redirect(url_for('index'))

@app.route('/right')
def right():
	global command
	command = 'RIGHT'
	sendCommand()
	log()
	return redirect(url_for('index'))

@app.route('/backward')
def backward():
	global command
	command = 'BACKWARD'
	sendCommand()
	log()
	return redirect(url_for('index'))

@app.route('/speedplus')
def speedplus():
	global speed
	if speed < 40:
		speed += 5
	return redirect(url_for('index'))

@app.route('/speedminus')
def speedminus():
	global speed
	if speed > 15:
		speed -= 5
	return redirect(url_for('index'))

@app.route('/angleplus')
def angleplus():
	global angle
	if angle < 180:
		angle += 15
	return redirect(url_for('index'))

@app.route('/angleminus')
def angleminus():
	global angle
	if angle > 0:
		angle -= 15
	return redirect(url_for('index'))

@app.route('/trackon')
def trackon():
	global track
	track = 1
	return redirect(url_for('index'))

@app.route('/trackoff')
def trackoff():
	global track
	track = 0
	return redirect(url_for('index'))

@app.route('/detecton')
def detecton():
	global detect
	detect = 1
	return redirect(url_for('index'))

@app.route('/detectoff')
def detectoff():
	global detect
	detect = 0
	return redirect(url_for('index'))

@app.route('/robot1')
def robot1():
	global robot
	robot = 'robot1'
	return redirect(url_for('index'))

@app.route('/robot2')
def robot2():
	global robot
	robot = 'robot2'
	return redirect(url_for('index'))

@app.route('/robot3')
def robot3():
	global robot
	robot = 'robot3'
	return redirect(url_for('index'))

@app.route('/robot4')
def robot4():
	global robot
	robot = 'robot4'
	return redirect(url_for('index'))

@app.route('/robot5')
def robot5():
	global robot
	robot = 'robot5'
	return redirect(url_for('index'))



if __name__ == '__main__':
	app.run(host='0.0.0.0', port=8800, debug=True)
