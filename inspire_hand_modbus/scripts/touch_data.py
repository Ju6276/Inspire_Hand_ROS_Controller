import time
from pymodbus.client import ModbusTcpClient
from pymodbus.pdu import ExceptionResponse

# define Modbus TCP related parameters
MODBUS_IP = "192.168.11.210"
MODBUS_PORT = 6000

# define the address range of each part of data
TOUCH_SENSOR_BASE_ADDR_PINKY = 3000  # pinky
TOUCH_SENSOR_END_ADDR_PINKY = 3369

TOUCH_SENSOR_BASE_ADDR_RING = 3370  # ring
TOUCH_SENSOR_END_ADDR_RING = 3739

TOUCH_SENSOR_BASE_ADDR_MIDDLE = 3740  # middle
TOUCH_SENSOR_END_ADDR_MIDDLE = 4109

TOUCH_SENSOR_BASE_ADDR_INDEX = 4110  # index
TOUCH_SENSOR_END_ADDR_INDEX = 4479

TOUCH_SENSOR_BASE_ADDR_THUMB = 4480  # thumb
TOUCH_SENSOR_END_ADDR_THUMB = 4899

TOUCH_SENSOR_BASE_ADDR_PALM = 4900  # palm
TOUCH_SENSOR_END_ADDR_PALM = 5123

# the maximum number of registers read by Modbus
MAX_REGISTERS_PER_READ = 125


def read_register_range(client, start_addr, end_addr):
    """
    Batch read the register data in the specified address range.
    """
    register_values = []  
    # segment read the register
    for addr in range(start_addr, end_addr + 1, MAX_REGISTERS_PER_READ * 2):

        current_count = min(MAX_REGISTERS_PER_READ, (end_addr - addr) // 2 + 1)


        response = client.read_holding_registers(address=addr, count=current_count)

        if isinstance(response, ExceptionResponse) or response.isError():
            print(f"read register {addr} failed: {response}")
            register_values.extend([0] * current_count)  
        else:
            register_values.extend(response.registers) 

    return register_values


def read_multiple_registers():
    client = ModbusTcpClient(MODBUS_IP, port=MODBUS_PORT)
    client.connect()

    try:
        while True:  
            start_time = time.time()  

            # read the data of each part
            pinky_register_values = read_register_range(
                client,
                TOUCH_SENSOR_BASE_ADDR_PINKY,
                TOUCH_SENSOR_END_ADDR_PINKY
            )

            ring_register_values = read_register_range(
                client,
                TOUCH_SENSOR_BASE_ADDR_RING,
                TOUCH_SENSOR_END_ADDR_RING
            )

            middle_register_values = read_register_range(
                client,
                TOUCH_SENSOR_BASE_ADDR_MIDDLE,
                TOUCH_SENSOR_END_ADDR_MIDDLE
            )

            index_register_values = read_register_range(
                client,
                TOUCH_SENSOR_BASE_ADDR_INDEX,
                TOUCH_SENSOR_END_ADDR_INDEX
            )

            thumb_register_values = read_register_range(
                client,
                TOUCH_SENSOR_BASE_ADDR_THUMB,
                TOUCH_SENSOR_END_ADDR_THUMB
            )

            palm_register_values = read_register_range(
                client,
                TOUCH_SENSOR_BASE_ADDR_PALM,
                TOUCH_SENSOR_END_ADDR_PALM
            )

            end_time = time.time()
            frequency = 1 / (end_time - start_time) 

            pinky_output_str = ", ".join(map(str, pinky_register_values))
            ring_output_str = ", ".join(map(str, ring_register_values))
            middle_output_str = ", ".join(map(str, middle_register_values))
            index_output_str = ", ".join(map(str, index_register_values))
            thumb_output_str = ", ".join(map(str, thumb_register_values))
            palm_output_str = ", ".join(map(str, palm_register_values))

            # print the data
            print(f"pinky data: {pinky_output_str}")
            print(f"ring data: {ring_output_str}")
            print(f"middle data: {middle_output_str}")
            print(f"index data: {index_output_str}")
            print(f"thumb data: {thumb_output_str}")
            print(f"palm data: {palm_output_str}")
            print(f"read frequency: {frequency:.2f} Hz")  

    finally:
        client.close()  


if __name__ == "__main__":
    read_multiple_registers()

