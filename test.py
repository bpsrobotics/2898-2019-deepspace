import math
import csv
import time

#Fuck pi
tau = math.pi * 2
#Time-step/dt
dt = 0.01

# Stall torque
kStallTorque = 2.402
# Stall current
kStallCurrent = 131
# Free speed
kFreeSpeed = 5330
# Free current
kFreeCurrent = 2.7

# Mass at the end of arm (if assuming rod has no weight), or center of mass
kMass = 1.032
# Length to center of mass (length of arm
kLength = 0.251
# Number of motors
kNumMotors = 1.0
# Gear ratio
kGearRatio = 21

# Motor coil resistance
kResistance = 12.0 / kStallCurrent
# Motor Velocity constant
Kv = ((kFreeSpeed / 60 * 2.0 * math.pi) / (12.0 - (kResistance * kFreeCurrent)))
# Motor torque constant (torque per amp)
Kt = (kNumMotors * kStallCurrent) / kStallCurrent
# Moment of inertia
kInertia = (1/3) * kMass * (kLength * kLength)

# Acceleration
acceleration = 0.0
# Velocity (starts at 0)
velocity = 0.0
# Position (starts at 0, need to define 0)
position = 0.0
#time to run
simTime = 10
# Goal (in meters)
goal = 3.14159

def getaccel(voltage):
    acceleration = ((kGearRatio * Kt)/(kResistance * kInertia)) * voltage - (kGearRatio * kGearRatio * Kt)/(Kv * kResistance * kInertia) * velocity
    return acceleration
with open('sim.csv', mode='w') as csv_file:
    fieldnames = ['position', 'time', 'goal', 'error']
    writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    writer.writerow(['position', 'time', 'goal', 'error'])
    time = 0
    error = 0
    while (time < simTime):
        lastError = error;
        error = goal - position
        kP = 40
        deriv = (error - lastError)/0.01
        kD = 0.08
        voltage = (kP * error) + (deriv * kD)
        if voltage < -12:
            voltage = -12
        elif voltage > 12:
            voltage = 12

        time = time + dt
        if error < (0.01 * goal):
            print(time)
        else:
            print("error" + str(error))

        writer.writerow([str(position), str(time), str(goal), str(error)])
        acceleration = getaccel(voltage)
        velocity += acceleration * kInertia * dt
        position += velocity * dt



