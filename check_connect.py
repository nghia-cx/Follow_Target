from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
import time
import argparse


def arm():
    print(f"Is armable: {vehicle.is_armable}")
    '''while vehicle.is_armable == False:
        print("Waiting to connect.....")
        time.sleep(1)'''
    print("Drone is armabled")

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while vehicle.armed== False:
        print("Waiting drone become arm")
        time.sleep(1)
    print("Vehicle is now arm!")
    
    return None

parser = argparse.ArgumentParser(description='commands')
parser.add_argument(u'--connect', default = '/dev/ttyACM0')
#parser.add_argument('--connect')
args = parser.parse_args()
baudrate=921600

##-- Connect to the drone
print ('Connecting...')
vehicle = connect(args.connect, baud=baudrate)

print(vehicle)
print(vehicle.mode)
print(vehicle.system_status)
print(vehicle.gps_0)
print(vehicle.ekf_ok)
#vehicle.parameters['ARMING_CHECK'] = 0
arm()
print("End Fly")

