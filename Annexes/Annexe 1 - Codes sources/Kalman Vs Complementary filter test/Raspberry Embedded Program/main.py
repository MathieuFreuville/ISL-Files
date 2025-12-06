


import RPi.GPIO as GPIO
from IMU import MPU6050
from IMU_Test import MPUFilterTest
import time


''' ------------------------------------
GLOBAL VARIABLES
'''
MainLoopTimer = pow(10,7)                           # Timer duration in nanoseconds
SamplingFrequency= (pow(10,9) / MainLoopTimer )
dt = MainLoopTimer/pow(10,9)                        # Elapsed time in the main loop (assumed perfect at the first loop)

LeftEncoder = 0                         # Left encoder pulses counte
RightEncoder = 0                        # Right Encoder pulses counter

''' ------------------------------------
SUBFONCTIONS
'''
def interrupt_encoder(channel):

    global LeftEncoder
    global RightEncoder

    if channel == 29:
        LeftEncoder += 1
    elif channel == 31:
        RightEncoder += 1

    return

''' ------------------------------------
INIT LOOP GPIO
'''
GPIO.setmode(GPIO.BOARD)                                # Set GPIO numbering as connector numbering
#GPIO.setmode(GPIO.BCM)                                 # Set GPIO numbering as ARM chip numbering

GPIO.setup(29, GPIO.IN, GPIO.PUD_UP)         # Set Pin 29 (GPIO 5) as input, used for Left motor encoder
GPIO.setup(31, GPIO.IN, GPIO.PUD_UP)         # Set Pin 31 (GPIO 6) as input, used for Right motor encoder

GPIO.add_event_detect(29, edge = GPIO.RISING, callback = interrupt_encoder )      #debounce is in ms
GPIO.add_event_detect(31, edge = GPIO.FALLING, callback = interrupt_encoder, bouncetime = 1 )      #debounce is in ms

# Test GPIO
GPIO.setup(35, GPIO.IN, GPIO.PUD_DOWN)
GPIO.setup(37, GPIO.OUT)
# print('GPIO Mode:' + str(GPIO.getmode()))
# print( 'Etat PIN 29: '+ str(GPIO.input(29)))


#help(gpiod)
# chip = gpiod.Chip("/dev/gpiochip4")
# info = chip.get_info()
# print(f"{info.name} [{info.label}] ({info.num_lines} lines)")

''' ------------------------------------
INIT LOOP I2C
'''
'''sensor = MPU6050(dt)
sensor.set_accel_range(sensor.ACCEL_RANGE_2G)
sensor.set_gyro_range(sensor.GYRO_RANGE_250DEG)
sensor.set_filter_range(sensor.FILTER_BW_256)
sensor.calibrate_gyros()
sensor.initialize_gyros_angles()'''

Test = MPUFilterTest(timer = pow(10,7), simulation_time =20)

# Test.test_acceleros()
#Test.test_gyro_drift()
#Test.test_lowpass_filter()
# Test.test_gyro_drift_complementary()
# Test.compare_gyro_drift_complementary()
Test.compare_all_filters()
''' ------------------------------------
MAIN LOOP
'''
i=0

while True :

    # Timer Initialisation
    t0 = time.clock_gettime_ns(time.CLOCK_REALTIME)
    i +=1
    sensor.get_6axis(dt)
    if i == 100:
        print('Acc = ' + str(sensor.Acc_Angle_X) + ' Gyr = ' + str(sensor.Gyr_Angle_X) + ' Compl = ' + str(sensor.Comp_Angle_X)   )
        i = 0

    while dt < MainLoopTimer:
        dt = time.clock_gettime_ns(time.CLOCK_REALTIME) - t0  # duration is integer with nanosecond resolutio
    dt /= pow(10,9)