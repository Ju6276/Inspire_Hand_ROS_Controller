#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
import rospy
from pymodbus.client import ModbusTcpClient
from pymodbus.pdu import ExceptionResponse
from inspire_hand_modbus.msg import get_force_act_1, get_angle_act_1, get_touch_act_1, set_angle_1, set_force_1, set_speed_1

# define Modbus TCP related parameters
MODBUS_IP = "192.168.11.210"
MODBUS_PORT = 6000

# define the address and finger ID of the force sensor data
FORCE_SENSOR_RANGES = {
    1: (1582,),  # Pinky
    2: (1584,),  # Ring Finger
    3: (1586,),  # Middle Finger
    4: (1588,),  # Index Finger
    5: (1590,),  # Thumb Flexion
    6: (1592,),  # Thumb Abduction
}

# define the address range of the touch sensor data
TOUCH_ACT_RANGES = {
    1: (3000, 3369),  # Pinky
    2: (3370, 3739),  # Ring Finger
    3: (3740, 4109),  # Middle Finger
    4: (4110, 4479),  # Index Finger
    5: (4480, 4899),  # Thumb
    7: (4900, 5123),  # Palm
}

# define the address and finger ID of the force setting data
FORCE_SET_RANGES = {
    1: (1498,),  # Pinky
    2: (1500,),  # Ring Finger
    3: (1502,),  # Middle Finger
    4: (1504,),  # Index Finger
    5: (1506,),  # Thumb Flexion
    6: (1508,),  # Thumb Abduction
}

# define the address and finger ID of the speed setting data
SPEED_SET_RANGES = {
    1: (1522,),  # Pinky
    2: (1524,),  # Ring Finger
    3: (1526,),  # Middle Finger
    4: (1528,),  # Index Finger
    5: (1530,),  # Thumb Flexion
    6: (1532,),  # Thumb Abduction
}

# define the address and finger ID of the angle sensor data
ANGLE_ACT_RANGES = {
    1: (1546,),  # Pinky
    2: (1548,),  # Ring Finger
    3: (1550,),  # Middle Finger
    4: (1552,),  # Index Finger
    5: (1554,),  # Thumb Flexion
    6: (1556,),  # Thumb Abduction
}

# define the address and finger ID of the angle setting data
ANGLE_SET_RANGES = {
    1: (1486,),  # Pinky
    2: (1488,),  # Ring Finger
    3: (1490,),  # Middle Finger
    4: (1492,),  # Index Finger
    5: (1494,),  # Thumb Flexion
    6: (1496,),  # Thumb Abduction
}

# create a dictionary mapping finger ID to name
FINGER_NAMES = {
    1: "Pinky",
    2: "Ring Finger",
    3: "Middle Finger",
    4: "Index Finger",
    5: "Thumb Flexion",
    6: "Thumb Abduction",
    7: "Palm",
}

def read_signed_register(client, address):
    """read a single register and return a signed integer value."""
    response = client.read_holding_registers(address=address, count=1)
    
    if isinstance(response, ExceptionResponse) or response.isError():
        rospy.logerr(f"read register {address} failed: {response}")
        return 0  # return 0 when read failed
    else:
        value = response.registers[0]
        if value > 32767:  
            value -= 65536  
        return value

def read_register_range(client, start_addr, end_addr):
    """batch read the register data in the specified address range."""
    register_values = []
    # segment read the register
    for addr in range(start_addr, end_addr + 1, 125 * 2):
        current_count = min(125, (end_addr - addr) // 2 + 1)
        response = client.read_holding_registers(address=addr, count=current_count)

        if isinstance(response, ExceptionResponse) or response.isError():
            rospy.logerr(f"read register {addr} failed: {response}")
            register_values.extend([0] * current_count)  # fill the register data when read failed
        else:
            register_values.extend(response.registers)  # add the successfully read register data

    return register_values

def read_touch_data(client):
    """read the touch data and return a dictionary (finger_id: touch_values)."""
    touch_data = {}

    for finger_id, (start_addr, end_addr) in TOUCH_ACT_RANGES.items():
        touch_values = read_register_range(client, start_addr, end_addr)
        touch_data[finger_id] = []

        for i in range(0, len(touch_values), 2):
            if i < len(touch_values):  
                combined_value = int(touch_values[i])  
                touch_data[finger_id].append(combined_value)

        touch_data[finger_id] = [int(value) for value in touch_data[finger_id]]

        rospy.loginfo(f"Finger ID: {finger_id}, Touch Values: {touch_data[finger_id]}")

    return touch_data

def publish_data():
    """read the data and publish the force, angle and touch data."""
    start_time = time.time()  

    # check the connection and publish the data
    if force_publisher.get_num_connections() > 0:
        # publish the force data
        force_data_msg = get_force_act_1() 
        force_data_msg.finger_ids = []  
        force_data_msg.force_values = []  
        force_data_msg.finger_names = []  

        for finger_id, (start_addr,) in FORCE_SENSOR_RANGES.items():
            force_value = read_signed_register(modbus_client, start_addr)
            force_data_msg.finger_ids.append(finger_id)
            force_data_msg.force_values.append(force_value)
            force_data_msg.finger_names.append(FINGER_NAMES.get(finger_id, "Unknown Finger"))

        force_publisher.publish(force_data_msg)
        rospy.loginfo(f"published force data, read frequency: {1 / (time.time() - start_time):.2f} Hz")

    if angle_publisher.get_num_connections() > 0:
        # publish the angle data
        angle_data_msg = get_angle_act_1()  
        angle_data_msg.finger_ids = []  
        angle_data_msg.angles = []  
        angle_data_msg.finger_names = []  

        for finger_id, (start_addr,) in ANGLE_ACT_RANGES.items():
            angle_value = read_signed_register(modbus_client, start_addr)  
            angle_data_msg.finger_ids.append(finger_id)
            angle_data_msg.angles.append(angle_value)  
            angle_data_msg.finger_names.append(FINGER_NAMES.get(finger_id, "Unknown Finger"))

        angle_publisher.publish(angle_data_msg)
        rospy.loginfo(f"published angle data, read frequency: {1 / (time.time() - start_time):.2f} Hz")

    if touch_publisher.get_num_connections() > 0:
        # publish the touch data
        touch_data = read_touch_data(modbus_client)
        touch_data_msg = get_touch_act_1()
        touch_data_msg.finger_ids = list(touch_data.keys())
        touch_data_msg.touch_values = []
        touch_data_msg.finger_names = []

        for finger_id in touch_data_msg.finger_ids:
            touch_data_msg.touch_values.extend(touch_data[finger_id])  # add the touch data of each finger
            touch_data_msg.finger_names.append(FINGER_NAMES.get(finger_id, "Unknown Finger"))

        touch_publisher.publish(touch_data_msg)
        rospy.loginfo(f"published touch data, read frequency: {1 / (time.time() - start_time):.2f} Hz")

def angle_callback(msg):
    """receive the message of setting the angle and write the corresponding register."""
    for finger_id, angle in zip(msg.finger_ids, msg.angles):
        if 0 <= angle <= 1000:  # ensure the angle is between 0 and 1000
            address = ANGLE_SET_RANGES.get(finger_id, (None,))[0]
            if address is not None:
                write_signed_register(modbus_client, address, angle)
            else:
                rospy.logwarn(f"no address found for finger ID {finger_id}")

def force_callback(msg):
    """receive the message of setting the force and write the corresponding register."""
    for finger_id, force in zip(msg.finger_ids, msg.forces):
        if 0 <= force <= 3000:  # ensure the force is between 0 and 3000
            address = FORCE_SET_RANGES.get(finger_id, (None,))[0]
            if address is not None:
                write_signed_register(modbus_client, address, force)
            else:
                rospy.logwarn(f"no address found for finger ID {finger_id}")

def speed_callback(msg):
    """receive the message of setting the speed and write the corresponding register."""
    for finger_id, speed in zip(msg.finger_ids, msg.speeds):
        if 0 <= speed <= 1000:  # ensure the speed is between 0 and 1000
            address = SPEED_SET_RANGES.get(finger_id, (None,))[0]
            if address is not None:
                write_signed_register(modbus_client, address, speed)
            else:
                rospy.logwarn(f"no address found for finger ID {finger_id}")

def write_signed_register(client, address, value):
    """write a signed integer value to the register."""
    if value < 0:
        value += 65536  # convert to unsigned value
    response = client.write_register(address, value)
    
    if isinstance(response, ExceptionResponse) or response.isError():
        rospy.logerr(f"write register {address} failed: {response}")

def data_reading_thread():
    """the thread of reading the data."""
    while not rospy.is_shutdown():
        publish_data()  # read and publish the data

def main():
    rospy.init_node("sensor_data_publisher", anonymous=True)

    # create the publisher
    global force_publisher, angle_publisher, touch_publisher
    force_publisher = rospy.Publisher("force_data", get_force_act_1, queue_size=10)  
    angle_publisher = rospy.Publisher("angle_data", get_angle_act_1, queue_size=10)  
    touch_publisher = rospy.Publisher("touch_data", get_touch_act_1, queue_size=10)  
    rospy.Subscriber("set_angle_data", set_angle_1, angle_callback)  
    rospy.Subscriber("set_force_data", set_force_1, force_callback)  
    rospy.Subscriber("set_speed_data", set_speed_1, speed_callback)  

    # create the Modbus TCP client
    global modbus_client
    modbus_client = ModbusTcpClient(MODBUS_IP, port=MODBUS_PORT)
    if not modbus_client.connect():
        rospy.logerr("failed to connect to Modbus server, please check the IP and port configuration")
        return

    rospy.loginfo("Modbus connected successfully")

    # start the data reading thread
    read_thread = threading.Thread(target=data_reading_thread)
    read_thread.start()

    try:
        rospy.spin()  # keep the node active
    except rospy.ROSInterruptException:
        rospy.loginfo("data publishing node is stopped manually")
    finally:
        modbus_client.close()

if __name__ == "__main__":
    main()

