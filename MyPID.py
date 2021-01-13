# BLIMP
# PID Control fro all

import math
import board
import busio
import adafruit_vl53l0x
import adafruit_bno055
import gpiozero
import time

def PID(p, currentValue, setPoint):
    prev_err = 0
    int_err = 0
    i = 0
    while i<100:
        err = setPoint - currentValue
        diff_err = err - prev_err
        int_err = int_err + err
        control = -p[0] * err - p[1] * (err - prev_err) - p[2] * int_err
        i = i + 1
        #time.sleep(1)
    #print ("PWM value", control)
    return control, err

def twiddle(tol=0.2):
    p = [0, 0, 0]
    dp = [1, 1, 1]
    control, best_err = PID(p, yawAngle, sp)

    it = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}".format(it, best_err))
        for i in range(len(p)):
            p[i] += dp[i]
            control, best_err = PID(p, yawAngle, sp)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                control, best_err = PID(p, yawAngle, sp)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return p

def motorT():
    a1 = ccw1.on()  # Sets clockwise direction pin ON
    b1 = cw1.off()  # Sets ccw direction pin ON
    a2 = ccw2.on()  # Sets cw direction pin ON
    b2 = cw2.off()  # Sets cw direction pin ON
    return a1, a2, b1, b2

def motorLR():
    aR = ccwR.on()
    bR = cwR.on()
    aL = ccwL.on()
    bL = cwL.on()
    return aR, bR, aL, bL


#Set up Motor
# For Motor 1:

ccw1 = gpiozero.OutputDevice(18)  # On/Off Output
cw1 = gpiozero.OutputDevice(23)  # On/Off Output

m1_speed = gpiozero.PWMOutputDevice(24)  # Set PWM pin

# For Motor 2:
cw2 = gpiozero.OutputDevice(17)  # On/Off Output
ccw2 = gpiozero.OutputDevice(27)  # On/Off Output

m2_speed = gpiozero.PWMOutputDevice(22)  # Set PWM pin

# For Motor 3 Left:
cwL = gpiozero.OutputDevice(16)  # On/Off Output
ccwL = gpiozero.OutputDevice(25)  # On/Off Output

mL_speed = gpiozero.PWMOutputDevice(12)  # Set PWM pin

# For Motor 4 Right:
ccwR = gpiozero.OutputDevice(5)  # On/Off Output
cwR = gpiozero.OutputDevice(6)  # On/Off Output

mR_speed = gpiozero.PWMOutputDevice(13)  # Set PWM pin

# Distance Sensor Setup

i2c = busio.I2C(board.SCL, board.SDA)
vl53 = adafruit_vl53l0x.VL53L0X(i2c)

# Gyro setup
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055(i2c)

# Set PID Variables

#params = [0.2, 0.5, 0.1] # Kp, Kd, Ki values
sp = 0 # Initialize Set Point to be zero

while True:
    # Getting the Distance from the sensor
    #D = vl53.range
    #print("Distance measured", D)

    # Yaw Angle Values
    E = sensor.euler
    print("Yaw Angle Measured:", E[0])


    yawAngle = E[0]  # change later
    #params, err = twiddle()
    #print("Final twiddle error = {}".format(err))
    if yawAngle == None:
        yawAngle = 0
    if yawAngle>360:
        yawAngle = 360

    if yawAngle>0 and yawAngle<180:
        sp = 0
    elif yawAngle>180 and yawAngle<360:
        sp = 360


    params, err = twiddle()
    print("Final twiddle error = {}".format(err))

    control, best_err = PID(params, yawAngle, sp)

    #control, err = PID(params, yawAngle, sp)
    print ("Control Value", control)
    print ("Error  Value", best_err)

    if control!=0:
        newControl = 10000/control
    else:
        newControl = 0

    print(newControl)

    if newControl>0.3:
        newControl = 0.3
    elif newControl<-0.3:
        newControl = -0.3
    print ("New Control", newControl)

    motorT()
    motorLR()

    #x = [x1, x2]
    #y = [1,1]
    #z = [-1,1]
    #x = [-1,1]*newControl + y*0.5
    #print ("x1 and x2", x)
    mL_speed.value = -1*newControl + 0.5
    mR_speed.value = newControl + 0.5

    l = mL_speed.value
    r = mR_speed.value

    print ("mL", l, "mR", r)

    time.sleep(1)
