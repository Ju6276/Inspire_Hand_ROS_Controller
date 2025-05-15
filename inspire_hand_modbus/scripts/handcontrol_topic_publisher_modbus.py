#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
from std_msgs.msg import String
from pymodbus.client import ModbusTcpClient
from pymodbus.pdu import ExceptionResponse

# define Modbus TCP related parameters
MODBUS_IP = "192.168.11.210"
MODBUS_PORT = 6000

# define the address range of each part of data
TOUCH_SENSOR_RANGES = {
    "pinky": (3000, 3369),
    "ring": (3370, 3739),
    "middle": (3740, 4109),
    "index": (4110, 4479),
    "thumb": (4480, 4899),
    "palm": (4900, 5123),
}

# the maximum number of registers read by Modbus
MAX_REGISTERS_PER_READ = 125


def read_register_range(client, start_addr, end_addr):
    """
    Batch read the register data in the specified address range.
    """
    register_values = []

    # segment read the register
    for addr in range(start_addr, end_addr + 1, MAX_REGISTERS_PER_READ * 2):
        # determine the number of registers in the current segment
        current_count = min(MAX_REGISTERS_PER_READ, (end_addr - addr) // 2 + 1)

        # batch read the register
        response = client.read_holding_registers(address=addr, count=current_count)

        if isinstance(response, ExceptionResponse) or response.isError():
            rospy.logerr(f"read register {addr} failed: {response}")
            register_values.extend([0] * current_count)  # use 0 to fill the failed data
        else:
            register_values.extend(response.registers)  # add the successfully read data

    return register_values


def main():
    rospy.init_node("handcontrol_publisher", anonymous=True)

    # create the publisher
    publisher = rospy.Publisher("touch_data", String, queue_size=10)

    # create the Modbus TCP client
    modbus_client = ModbusTcpClient(MODBUS_IP, port=MODBUS_PORT)
    if not modbus_client.connect():
        rospy.logerr("failed to connect to Modbus server, please check the IP and port configuration")
        return

    rospy.loginfo("Modbus connected successfully")

    # publish frequency
    rate = rospy.Rate(50)  # 50 Hz

    try:
        while not rospy.is_shutdown():
            start_time = time.time()

            touch_data = {}
            for part_name, (start_addr, end_addr) in TOUCH_SENSOR_RANGES.items():
                values = read_register_range(modbus_client, start_addr, end_addr)
                touch_data[part_name] = ", ".join(map(str, values))

            output = "\n".join(
                [f"{name} data: {data}" for name, data in touch_data.items()]
            )

            # publish message
            msg = String()
            msg.data = output
            publisher.publish(msg)

            # print the read frequency
            end_time = time.time()
            frequency = 1 / (end_time - start_time)
            rospy.loginfo(f"read frequency: {frequency:.2f} Hz")

            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Modbus publisher node is stopped manually")
    finally:
        modbus_client.close()


if __name__ == "__main__":
    main()

