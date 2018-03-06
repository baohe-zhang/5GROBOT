import RPi.GPIO as GPIO
import time 
import threading
#Alphabot
def init():

	GPIO.cleanup()
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)

	global IN1
	IN1=12
	GPIO.setup  (IN1, GPIO.OUT)
	GPIO.output (IN1, GPIO.LOW)

	global IN2
	IN2=13
	GPIO.setup  (IN2, GPIO.OUT)
	GPIO.output (IN2, GPIO.LOW)
	
	global IN3
	IN3=20
	GPIO.setup  (IN3, GPIO.OUT)
	GPIO.output (IN3, GPIO.LOW)
	
	global IN4
	IN4=21
	GPIO.setup  (IN4, GPIO.OUT)
	GPIO.output (IN4, GPIO.LOW)
	
	global PWMA
	ENA=6
	GPIO.setup(ENA, GPIO.OUT)
	PWMA=GPIO.PWM(ENA,1000)
	
	global PWMB
	ENB=26
	GPIO.setup (ENB, GPIO.OUT)
	PWMB=GPIO.PWM (ENB, 1000)
	
	global sensorL
	sensorL=7
	GPIO.setup(sensorL, GPIO.IN)

	global sensorR
	sensorR=8
	GPIO.setup(sensorR, GPIO.IN)

	#Define Duty Cycle
	PWMA.start (90)
	PWMB.start (90)
	
	#Speed
	global sampleL
	sampleL=19
	global sampleR
	sampleR=19

	global countL
	countL=0
	global countR
	countR=0

	global LineControl
	LineControl=0
	global ini_linetracking
	ini_linetracking=0
	
	global dir_car
	dir_car=1

	global delay
	delay=0.15

	global trline
	
# Line Tracking

	global CS
	CS=5
	global Clock
	Clock=25
	global Address
	Address=24
	global DataOut
	DataOut=23

	GPIO.setup(Clock,GPIO.OUT)
	GPIO.setup(Address,GPIO.OUT)
	GPIO.setup(CS,GPIO.OUT)
	GPIO.setup(DataOut,GPIO.IN,GPIO.PUD_UP)

	global numSensors
	numSensors = 5
	global calibratedMin
	calibratedMin = [0] * numSensors
	global calibratedMax
	calibratedMax = [1023] * numSensors
	global last_value
	last_value = 0

	global speed

#   defines if the controlSpeed is applied
	global controlSpeed
	controlSpeed=0
#	It is used to re start or not the control speed 
	global ini_controlSpeed
	ini_controlSpeed=0

	global DutyCycle,DutyCycleL,DutyCycleR
	DutyCycle=40
	DutyCycleL=40
	DutyCycleR=40

	GPIO.remove_event_detect(sensorR)
	GPIO.remove_event_detect(sensorL)

	global command_prev,command_now
	command_prev='none'
	command_now='STOP'

#   max DutyCycle when car is turning and minimum value when turn command is activated
	global max_curva
#	Robot 3
#	max_curva=45
#	Robot 2
#	max_curva=45
#	Robot 1
	max_curva=47


	global time_goal
	time_goal=0.05
	global weight_wheel
	weight_wheel=300
	global max_correct
	max_correct=10

	global first_hole_L, first_hole_R

	global dif_time_L, dif_time_R
	dif_time_L=0.
	dif_time_R=0.

# obstacle
	global DR
	DR = 19
	GPIO.setup(DR, GPIO.IN, GPIO.PUD_UP)

	global DL
	DL = 16
	GPIO.setup(DL, GPIO.IN, GPIO.PUD_UP)

	GPIO.add_event_detect(DR, GPIO.BOTH, callback=DR_callback)
	GPIO.add_event_detect(DL, GPIO.BOTH, callback=DL_callback)

        global DR_flag, DL_flag
        DR_flag = False
        DL_flag = False

	
#	print "ROBOT INIT"

def set_DutyCycle():

	global DutyCycleL, DutyCycleR
	if DutyCycleL>=99:
		DutyCycleL=99
	elif DutyCycleR>=99:
		DutyCycleR=99
	if DutyCycleL<=0:
		DutyCycleL=0
	elif DutyCycleR<=0:
		DutyCycleR=0
	PWMA.ChangeDutyCycle (DutyCycleL)
	PWMB.ChangeDutyCycle (DutyCycleR)


def ADC(A):
       # print "Battery Level "
	value = 0
	GPIO.output(CS, GPIO.LOW)
	for i in range (0,4):
		#sent 4-bit Address
		if(((A) >> (3 - i)) & 0x01):
			GPIO.output(Address,GPIO.HIGH)
		#print "Address 1"
		else:
			GPIO.output(Address,GPIO.LOW)
			#print "Address 0"
			#read MSB 4-bit data
		value <<= 1
		#print ("i",i)
		#print ("Value:", value)
		if(GPIO.input(DataOut)):
			value |= 0x01
			#print ("Value 1")
		GPIO.output(Clock,GPIO.HIGH)
		GPIO.output(Clock,GPIO.LOW)
	for i in range(0,6):
#print ("i:",i)
#read LSB 8-bit data
		value <<= 1
#print ("Value", value)
		if(GPIO.input(DataOut)):
			value |= 0x01
#print ("Value",value)
		GPIO.output(Clock,GPIO.HIGH)
		GPIO.output(Clock,GPIO.LOW)
#no mean ,just delay
	for i in range(0,6):
		GPIO.output(Clock,GPIO.HIGH)
		GPIO.output(Clock,GPIO.LOW)
#time.sleep(0.0001)
	GPIO.output(CS,GPIO.HIGH)
#print value
	return value

#   Line tracking part

#class TRSensor(object):


	"""
	Reads the sensor values into an array. There *MUST* be space
	for as many values as there were sensors specified in the constructor.
	Example usage:
	unsigned int sensor_values[8];
	sensors.read(sensor_values);
	The values returned are a measure of the reflectance in abstract units,
	with higher values corresponding to lower reflectance (e.g. a black
	surface or a void).
	
	"""

def AnalogRead():
	value = [0,0,0,0,0,0]
	#Read Channel0~channel4 AD value
	for j in range(0,6):
		GPIO.output(CS, GPIO.LOW)
		for i in range(0,4):
		#sent 4-bit Address
			if(((j) >> (3 - i)) & 0x01):
				GPIO.output(Address,GPIO.HIGH)
			else:
				GPIO.output(Address,GPIO.LOW)
                #read MSB 4-bit data
                	value[j] <<= 1
                	if(GPIO.input(DataOut)):
				value[j] |= 0x01
			GPIO.output(Clock,GPIO.HIGH)
			GPIO.output(Clock,GPIO.LOW)
		for i in range(0,6):
                #read LSB 8-bit data
			value[j] <<= 1
			if(GPIO.input(DataOut)):
				value[j] |= 0x01
			GPIO.output(Clock,GPIO.HIGH)
			GPIO.output(Clock,GPIO.LOW)
            #no mean ,just delay
		for i in range(0,6):
			GPIO.output(Clock,GPIO.HIGH)
			GPIO.output(Clock,GPIO.LOW)
#           time.sleep(0.0001)
		GPIO.output(CS,GPIO.HIGH)
	return value[1:]

def calibrate():
	global calibratedMin
	global calibratedMax
	
	cal=0
#   manual calibration for new surfae	
	if (cal==1):
		max_sensor_values = [0]*numSensors
		min_sensor_values = [0]*numSensors
		for j in range(0,10):
		
			sensor_values = AnalogRead();
			
			for i in range(0,numSensors):
			
				# set the max we found THIS time
				if((j == 0) or max_sensor_values[i] < sensor_values[i]):
					max_sensor_values[i] = sensor_values[i]

				# set the min we found THIS time
				if((j == 0) or min_sensor_values[i] > sensor_values[i]):
					min_sensor_values[i] = sensor_values[i]

		# record the min and max calibration values
		for i in range(0,numSensors):
			if(min_sensor_values[i] > calibratedMin[i]):
				calibratedMin[i] = min_sensor_values[i]
			if(max_sensor_values[i] < calibratedMax[i]):
				calibratedMax[i] = max_sensor_values[i]
	
	else: 
		# Min=730 Max=350 default linea estrecha
		calibratedMin[0]=1000
		calibratedMin[1]=1000
		calibratedMin[2]=1000
		calibratedMin[3]=1000
		calibratedMin[4]=1000
		calibratedMax[0]=400
		calibratedMax[1]=400
		calibratedMax[2]=400
		calibratedMax[3]=400
		calibratedMax[4]=400


def readCalibrated():
	value = 0
		#read the needed values
	sensor_values = AnalogRead();
	for i in range (0,numSensors):
		denominator = calibratedMax[i] - calibratedMin[i]
		if(denominator != 0):
			value = (sensor_values[i] - calibratedMin[i])* 1000 / denominator
		if(value < 0):
			value = 0
		elif(value > 1000):
			value = 1000
		sensor_values[i] = value
#	print("readCalibrated",sensor_values)
	return sensor_values
			
	"""
	Operates the same as read calibrated, but also returns an
	estimated position of the robot with respect to a line. The
	estimate is made using a weighted average of the sensor indices
	multiplied by 1000, so that a return value of 0 indicates that
	the line is directly below sensor 0, a return value of 1000
	indicates that the line is directly below sensor 1, 2000
	indicates that it's below sensor 2000, etc.  Intermediate
	values indicate that the line is between two sensors.  The
	formula is:

	   0*value0 + 1000*value1 + 2000*value2 + ...
	   --------------------------------------------
			 value0  +  value1  +  value2 + ...

	By default, this function assumes a dark line (high values)
	surrounded by white (low values).  If your line is light on
	black, set the optional second argument white_line to true.  In
	this case, each sensor value will be replaced by (1000-value)
	before the averaging.
	"""

def readLine():

	global last_value

	white_line=1
	sensor_values = readCalibrated()
	avg = 0
	sum = 0
	on_line = 0
	
	for i in range(0,numSensors):
		value = sensor_values[i]
		if(white_line):
			value = 1000-value
		# keep track of whether we see the line at all
		# value default 200
		if(value > 200):
			on_line = 1
		# only average in values that are above a noise threshold
		if(value > 50):
			avg += value * (i * 1000);  # this is for the weighted total,
			sum += value;                  #this is for the denominator 
	
	if(on_line != 1):
		# If it last read to the left of center, return 0.
		if(last_value < (numSensors - 1)*1000/2):
			#print("left")
			return 0;
		# If it last read to the right of center, return the max.
		else:
			#print("right")
			return (numSensors - 1)*1000

	last_value = avg/sum
		
	return last_value

class LineTrackerThread_ori(threading.Thread):

        def __init__(self):
                threading.Thread.__init__(self)
                self._running = True

        def terminate(self):
                self._running = False

        def run(self):
                print 'line thread start!!!'
                maximum = 40
                integral = 0
                last_proportional = 0
                calibrate()
                while self._running:
                        time.sleep(0.002)
                        position = readLine()

                        proportional = position - 2000
                        derivative = proportional - last_proportional
                        integral += proportional
                        last_proportional = proportional

                        power_difference = proportional/25 + derivative/100

                        if (power_difference > maximum):
                                power_difference = maximum
                        if (power_difference < - maximum):
                                power_difference = - maximum

                        if (power_difference < 0):
                                PWMB.ChangeDutyCycle(maximum + power_difference)
                                PWMA.ChangeDutyCycle(maximum)
                        else:
                                PWMB.ChangeDutyCycle(maximum)
                                PWMA.ChangeDutyCycle(maximum - power_difference)
                print 'line thread stop!!!'
                return

class LineTrackerThread(threading.Thread):

        def __init__(self):
                threading.Thread.__init__(self)
                self._running = True

        def terminate(self):
                self._running = False
 
        def run(self):
                print 'line thread start'
                
                global LineControl
                global DutyCycleL
                global DutyCycleR

        #	maximum=DutyCycleR
                maximum=60
                integral = 0
                last_proportional = 0

        #	time.sleep(0.5)
        #	for i in range(0,400):
                calibrate()
        #	time.sleep(0.5)	

        #	print(calibratedMin)
        #	print(calibratedMax)

                while LineControl and self._running:
                        time.sleep(0.002)
                        position = readLine()
                        # The "proportional" term should be 0 when we are on the line.
                        proportional = position - 2000
                        # Compute the derivative (change) and integral (sum) of the position.
                        derivative = proportional - last_proportional
                        integral += proportional
                        
                        # Remember the last position.
                        last_proportional = proportional
          
                        '''
                        // Compute the difference between the two motor power settings,
                        // m1 - m2.  If this is a positive number the robot will turn
                        // to the right.  If it is a negative number, the robot will
                        // turn to the left, and the magnitude of the number determines
                        // the sharpness of the turn.  You can adjust the constants by which
                        // the proportional, integral, and derivative terms are multiplied to
                        // improve performance.
                        '''
        # default proportional 25 derivative 100 integral 1000
                        power_difference = proportional/13 + derivative/20 # + integral/1000;  
        #		print("DutyLine",DutyCycleR,DutyCycleL)
        #		print("Powerdif",proportional, derivative, power_difference) 
                        if (power_difference > maximum):
                                power_difference = maximum
                        if (power_difference < - maximum):
                                power_difference = - maximum

                        if (power_difference < 0):
        #			print('Duty1',DutyCycleL,DutyCycleR + power_difference)
                                PWMA.ChangeDutyCycle (DutyCycleL)
                                if (DutyCycleR+power_difference<0):
                                        PWMB.ChangeDutyCycle (0)
                                else:
                                        PWMB.ChangeDutyCycle (DutyCycleR + power_difference)
                        else:
        #			print('Duty2', DutyCycleL - power_difference,DutyCycleR)
                                if (DutyCycleL - power_difference<0):
                                        PWMA.ChangeDutyCycle (0)	
                                else:
                                        PWMA.ChangeDutyCycle (DutyCycleL - power_difference)
                                PWMB.ChangeDutyCycle (DutyCycleR)

                print 'line thread terminate'


def set_start(dir):
	global start_L
	global start_R
	if (dir==0):	
		start_L=time.time()
	if (dir==1):	
		start_R=time.time()

def set_end(dir):
	global end_L
	global end_R
	if (dir==0):	
		end_L=time.time()
	if (dir==1):	
		end_R=time.time()

def increaseHoleL(c):
	global countL
	global controlSpeed
	global DutyCycleL
	countL=countL+1
	if (countL==sampleL and controlSpeed==0):
		countL=0
		GPIO.remove_event_detect(sensorL)
		DutyCycleL=DutyCycleL_prev
#		controlSpeed=1
#		if dir_car==1:
#			forward ()
#		elif dir_car==0:
#			backward ()
#		velocidad()
#		reset_sensors()
		stop()

def increaseHoleR(c):
	global countR
	global controlSpeed
	global DutyCycleR
	countR=countR+1
	if(countR==sampleR and controlSpeed==0):
		countR=0
		GPIO.remove_event_detect(sensorR)
		DutyCycleR=DutyCycleR_prev
#		controlSpeed=1
#		if dir_car==1:
#			forward ()
#		elif dir_car==0:
#			backward ()
#		velocidad()
#		reset_sensors()
		stop()

def holeCounter(option, angle):

	if option==0:#Left
		global sampleL
		sampleL=angle
		print('turn_sample L', sampleL)
		GPIO.add_event_detect(sensorL,GPIO.RISING, callback=increaseHoleL)	
		
	else: #Right1
		global sampleR
		sampleR=angle
		print('turn_sample R', sampleR)
		GPIO.add_event_detect(sensorR, GPIO.RISING, callback=increaseHoleR)

def control_speed_time_L(c):
	global DutyCycleL, dif_time_L, countL, max_correct,time_wheel_L,first_hole_L
	set_end(0)
	countL=countL+1
	time_wheel_L=end_L-start_L
	dif_time_L=time_wheel_L-time_goal
	div_time=1.
	print('Duty_L',DutyCycleL,int(round(1000*dif_time_L)),int(round(1000*time_wheel_L)),int(round(1000*time_goal)))
	if LineControl==1:
		dif_time=min(dif_time_L,dif_time_R)
		div_time=abs(min(time_wheel_L,time_wheel_R)/max(time_wheel_L,time_wheel_R))
#		print('Line_control_L',dif_time,div_time)
	else:
		dif_time=dif_time_L
	correct=int(round(dif_time*weight_wheel))
	if first_hole_L==0:
		max_value=max_correct
	else:
		max_value=int(round(max_correct/2.))	

	first_hole_L=0	

	if correct>=max_value:
		correct=max_value
	if correct<=-max_value:
		correct=-max_value
	DutyCycleL=DutyCycleL+correct
	if DutyCycleL>=99:
		DutyCycleL=99
	if DutyCycleL<=0:
		DutyCycleL=0
	if div_time<=0.5:
		print('turning limited Duty')
		if DutyCycleL>=max_curva:
			DutyCycleL=max_curva

	PWMA.ChangeDutyCycle (DutyCycleL)
	set_start(0)

def control_speed_time_R(c):
	global DutyCycleR, dif_time_R, countR, max_correct,time_wheel_R, first_hole_R
	set_end(1)
	countR=countR+1
	time_wheel_R=end_R-start_R
	dif_time_R=time_wheel_R-time_goal 
	div_time=1.

	print('Duty_R',DutyCycleR,int(round(1000*dif_time_R)),int(round(1000*time_wheel_R)),int(round(1000*time_goal)))

	if LineControl==1:
		dif_time=min(dif_time_L,dif_time_R)
		div_time=abs(min(time_wheel_L,time_wheel_R)/max(time_wheel_L,time_wheel_R))
#		print('Line_control_R',dif_time,div_time)
	else:
		dif_time=dif_time_R
	correct=int(round(dif_time*weight_wheel))

	if first_hole_R==0:
		max_value=max_correct
	else:
		max_value=int(round(max_correct/2.))

	first_hole_R=0
	if correct>=max_value:
		correct=max_value
	if correct<=-max_value:
		correct=-max_value

	DutyCycleR=DutyCycleR+correct
	if DutyCycleR>=99:
		DutyCycleR=99
	if DutyCycleR<=0:
		DutyCycleR=0
	if div_time<=0.4:
		print('turning limited Duty')
		if DutyCycleR>=max_curva:
			DutyCycleR=max_curva

	PWMB.ChangeDutyCycle (DutyCycleR)

	set_start(1)


def speedControl(delay):
#	print "speed control"
	global countR
	global countL
	global DutyCycleL
	global DutyCycleR
	global DutyCycle
	global controlSpeed
	
	if (controlSpeed==1):	
		print ('speed control: CountR CountL', countR, countL)
		if (countR==0 and countL==0):
			print('car stopped - push')
			if LineControl==1:
				DutyCycleL=int(round(DutyCycleL*1.15))
				DutyCycleR=int(round(DutyCycleR*1.15))
#				DutyCycleL=max_curva
#				DutyCycleR=max_curva
			else:
				DutyCycleL=int(round(DutyCycleL*1.2))
				DutyCycleR=int(round(DutyCycleR*1.2))
#			print ('duty L-R',DutyCycleL, DutyCycleR)

			if LineControl==1:
				if DutyCycleL>=max_curva:
					DutyCycleL=max_curva
				if DutyCycleR>=max_curva:
					DutyCycleR=max_curva
			print('push power',DutyCycleL,DutyCycleR) 
		if (countR==0 and countL!=0 and LineControl==0):
			print('car spinning - push R', DutyCycleR)
			DutyCycleR=int(round(DutyCycleR*1.2))
#			print ('duty L-R',DutyCycleL, DutyCycleR)
		if (countL==0 and countR!=0 and LineControl==0):
			print('car spinning - push L', DutyCycleL)
			DutyCycleL=int(round(DutyCycleL*1.2))

		print ('duty L-R',DutyCycleL, DutyCycleR)

		set_DutyCycle()
	
		countL=0
		countR=0
		timeout=threading.Timer(delay,speedControl,[delay])
		timeout.start()
     

def forward():
	GPIO.output (IN1, GPIO.LOW)
	GPIO.output (IN2, GPIO.HIGH)
	GPIO.output (IN3, GPIO.HIGH)
	GPIO.output (IN4, GPIO.LOW)

def backward():
	GPIO.output (IN1, GPIO.HIGH)
	GPIO.output (IN2, GPIO.LOW)
	GPIO.output (IN3, GPIO.LOW)
	GPIO.output (IN4, GPIO.HIGH)

def left():
	if dir_car==1:
		GPIO.output (IN1, GPIO.LOW)
		GPIO.output (IN2, GPIO.HIGH)
	elif dir_car==0:
		GPIO.output (IN1, GPIO.HIGH)
		GPIO.output (IN2, GPIO.LOW)
	GPIO.output (IN3, GPIO.LOW)
	GPIO.output (IN4, GPIO.LOW)

def right():
	GPIO.output (IN1, GPIO.LOW)
	GPIO.output (IN2, GPIO.LOW)
	if dir_car==0:
		GPIO.output (IN3, GPIO.LOW)
		GPIO.output (IN4, GPIO.HIGH)
	elif dir_car==1:
		GPIO.output (IN3, GPIO.HIGH)
		GPIO.output (IN4, GPIO.LOW)

def stop ():
	GPIO.output (IN1, GPIO.LOW)
	GPIO.output (IN2, GPIO.LOW)
	GPIO.output (IN3, GPIO.LOW)
	GPIO.output (IN4, GPIO.LOW)

def transi_velocidad (CL,CR):

	PWMA.ChangeDutyCycle (50+CL)
	PWMB.ChangeDutyCycle (50+CR)
	time.sleep(0.05)


def ini_velocidad(speed):

	global DutyCycle, DutyCycleR, DutyCycleL
	global delay
	global time_goal
	global weight_wheel
	global first_hole_L, first_hole_R

	first_hole_L=1
	first_hole_R=1

	if (speed<15 and speed!=0):
		speed=15

	BatGood=1
	i=0
	battery_mean=0
	while BatGood:
		battery=ADC(10)
		#battery=800
		#print ("BAT", battery)
		if 500<battery<1000:
			i=i+1
			battery_mean=battery+battery_mean
#print ("BatMean", battery_mean)
			if i==5:
				battery_mean=battery_mean/5
				BatGood=0
				break
 
	print ("Battery Level Init", battery_mean)
	if battery_mean<600:
		stop()
		print "Battery CHANGE"
	elif battery<700:
		if dir_car==1:
			A=0.015
			B=-0.017
			C=30
			
		else:
			A=0.014
			B=-0.013
			C=35
		DutyCycle=int(round(A*speed*speed+speed*B+C))
		print("LOW speed Initial DutyCycle",DutyCycle)
	else:
		if dir_car==1:
			A=0.015
			B=-0.017
			C=28
			
		else:
			A=0.014
			B=-0.013
			C=33
		DutyCycle=int(round(A*speed*speed+speed*B+C))
		print("High speed Initial DutyCycle",DutyCycle)

	time_goal=1./speed

	weight_wheel=50+10*speed
	if (speed<=20):
		max_correct=5
	elif (speed>20 and speed<=25):
		max_correct=7
	elif speed>=30:
		max_correct=11
	else:
		max_correct=8
	
	print('weight',weight_wheel)
	if (LineControl==1):
		weight_wheel=int(round(weight_wheel/2.))
		max_correct=int(round(max_correct/1.9))

#	weight_wheel=1000

#	choose calibrateion for each car
#	Robot 2
#	cal_L=3
#	Robot 3
#	cal_L=5
#	Robot 1
#	cal_R=2
	
	if (dir_car==1):
		cal_R=1
	else:
		cal_R=5
#	cal_L=0

	DutyCycleL=DutyCycle
	DutyCycleR=DutyCycle+cal_R

	print ('pow_ini',DutyCycleR, DutyCycleL)
	if (command_now!=command_prev) and (speed<29):
		transi_velocidad(0,cal_R)	
	set_DutyCycle()

def velocidad():
	global delay
	timeout=threading.Timer(delay,speedControl,[delay])
	if (controlSpeed==0):
#		print "Stop GPIO detect"
		GPIO.remove_event_detect(sensorR)
		GPIO.remove_event_detect(sensorL)
		timeout.cancel()
#		print "Timer stop"
	else:
#		print "Stop GPIO detect"
		GPIO.remove_event_detect(sensorR)
		GPIO.remove_event_detect(sensorL)
#		print "Start GPIO detect"
		set_start(0)
		set_start(1)
		GPIO.add_event_detect(sensorL,GPIO.RISING, callback=control_speed_time_L)
		GPIO.add_event_detect(sensorR,GPIO.RISING, callback=control_speed_time_R)
#		print "Timer Start"
		timeout.start()


def reset_sensors():
	GPIO.remove_event_detect(sensorR)
	GPIO.remove_event_detect(sensorL)

def commandEval ( command, speed, angle, TrackStatus ):
	global LineControl
	global dir_car
	global controlSpeed
	global ini_controlSpeed, ini_linetracking
	global countR, countL
	global command_prev,command_now
	global DutyCycleL, DutyCycleR, DutyCycleL_prev, DutyCycleR_prev
	global delay
	global trline
	
	# for obstacle BH
	global command_current, speed_current, angle_current, TrackStatus_current
	if str(command) != 'STOP':
                command_current = str(command)
                speed_current = speed
                angle_current = angle
                TrackStatus_current = TrackStatus
                
#	default 4.736
	angle=round(angle/4.5 )
	command_prev=command_now
	command_now=str(command)


#	if (ini_linetracking==1):
#		trline.join()
	
#	t=threading.Thread(target=LineTracker)

	if str(command) == "FORWARD":
		print ("Forward",command_prev,command_now)
		LineControl=TrackStatus
		dir_car=1
		controlSpeed=1
		ini_controlSpeed=1
		if (LineControl==1 and speed<=20):
			delay=0.15
			ini_linetracking=1			
		else:
			delay=0.1
		forward ()
		if ini_controlSpeed==1:
			ini_velocidad(speed)
		# from stop to forward will call velocidad, change speed will not. BH
		if command_now!=command_prev:
			velocidad()
			ini_controlSpeed=1
		if LineControl==1:
#			trline=threading.Thread(target=LineTracker)
#			t.join()
                        trline = LineTrackerThread()
			trline.start()
#			t.join()

	elif str(command) == "BACKWARD":
		print "Backward"	
		LineControl=0		
#		ini_linetracking=0
		delay=0.1
		dir_car=0
		controlSpeed=1
		backward ()
		ini_velocidad(speed)		
		velocidad ()

	elif str(command) == "LEFT":
		controlSpeed=0
		LineControl=0
		ini_linetracking=0
		velocidad()
#		reset_sensors()
		countR=0
		countL=0
		print ('turn_left_holes',angle)
		ini_velocidad (speed)		
		DutyCycleR_prev=DutyCycleR
		DutyCycleL_prev=DutyCycleL
		if (DutyCycleL<=max_curva or DutyCycleR<=max_curva):
			DutyCycleL=max_curva
			DutyCycleR=max_curva 
		set_DutyCycle()		
		if dir_car==1:
			left ()
			holeCounter(0,angle)
		elif dir_car==0:
			right ()
			holeCounter(1,angle)
		print "Left"	

	elif str(command) == "RIGHT":
		controlSpeed=0
		LineControl=0
		ini_linetracking=0
		velocidad()
#		reset_sensors ()
		countR=0
		countL=0
		print('turn_right_holes',angle)
		ini_velocidad (speed)
		DutyCycleR_prev=DutyCycleR
		DutyCycleL_prev=DutyCycleL
		if (DutyCycleR<=max_curva or DutyCycleL<=max_curva):
			DutyCycleL=max_curva
			DutyCycleR=max_curva 
		set_DutyCycle()
		if dir_car==1:
			right ()
		elif dir_car==0:
			left ()
		print "Right"	
		holeCounter(dir_car,angle)

	elif str(command) == "STOP":
#		t.stop()
		controlSpeed=0
		ini_linetracking=0		
		velocidad()
#		reset_sensors()
		stop ()
		if trline.is_alive():
                        trline.terminate()
		print "Stop"	
	else:
#		t.stop()
		print "Command not valid- ROBOT NOT MOVE"
		controlSpeed=0
		ini_linetracking=0
		velocidad()
#		reset_sensors()
		stop()



def DR_callback(c):
        global DR_flag, trline

        # command_prev != 'none' measn the robot has executed at least one valid command
        if GPIO.input(DR) == 0 and command_prev != 'none':
                commandEval('STOP', 0, 0, 0)
                DR_flag = True          # reduce bounce
        elif DR_flag and GPIO.input(DL) == 1 and command_prev != 'none':
                print 'Continue'
                print 'command:', command_current
                print 'speed:', speed_current
                print 'angle_current:', angle_current
                print 'TrackStatus:', TrackStatus_current
                commandEval(command_current, speed_current, angle_current, TrackStatus_current)
                DR_flag = False

def DL_callback(c):
        global DL_flag, trline
        
        if GPIO.input(DL) == 0 and command_prev != 'none':
                commandEval('STOP', 0, 0, 0)
                DL_flag = True  
        elif DL_flag and GPIO.input(DR) == 1 and command_prev != 'none':
                print 'Continue'
                print 'command:', command_current
                print 'speed:', speed_current
                print 'angle_current:', angle_current
                print 'TrackStatus:', TrackStatus_current
                commandEval(command_current, speed_current, angle_current, TrackStatus_current)
                DL_flag = False 

                
