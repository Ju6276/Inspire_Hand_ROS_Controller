#include <ros/ros.h>
#include <inspire_hand_modbus/set_id.h>
#include <inspire_hand_modbus/set_redu_ratio.h>
#include <inspire_hand_modbus/set_pos.h>
#include <cstdlib>
#include "hand_control.h"
#include <modbus.h> 

// global variable of Modbus client
modbus_t *ctx;

// send Modbus request
void sendModbusPositionCommand(int pos0, int pos1, int pos2, int pos3, int pos4, int pos5) {
    // start address
    uint16_t startAddress = 0x05CE; // modify the start address according to the need
    uint16_t registerCount = 6;    // 6 registers

    // create the data to send
    uint16_t data[6] = {pos0, pos1, pos2, pos3, pos4, pos5};

    // send data
    int rc = modbus_write_registers(ctx, startAddress, registerCount, data);
    if (rc == -1) {
        ROS_ERROR("Failed to write registers: %s", modbus_strerror(errno));
    } else {
        ROS_INFO("Successfully wrote %d positions to Modbus starting from address 0x%04X", registerCount, startAddress);
    }

    // read the response (if needed)
    uint8_t response[256];
    int response_length = modbus_receive(ctx, response);
    if (response_length >= 0) {
        ROS_INFO("Received response from Modbus server");
    } else {
        ROS_ERROR("Failed to read response from Modbus server: %s", modbus_strerror(errno));
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hand_control_client");
    ros::NodeHandle nh;

    // create the Modbus TCP client
    ctx = modbus_new_tcp("192.168.11.210", 6000); 
    if (modbus_connect(ctx) == -1) {
        ROS_ERROR("Unable to connect to Modbus server: %s", modbus_strerror(errno));
        return -1;
    }

    // create the service client
    ros::ServiceClient setIdClient = nh.serviceClient<inspire_hand_modbus::set_id>("inspire_hand_modbus/set_id");
    ros::ServiceClient setRatioClient = nh.serviceClient<inspire_hand_modbus::set_redu_ratio>("inspire_hand_modbus/set_redu_ratio");
    ros::ServiceClient setPosClient = nh.serviceClient<inspire_hand_modbus::set_pos>("inspire_hand_modbus/set_pos");

    // create the service request
    inspire_hand_modbus::set_id setIdSrv;
    inspire_hand_modbus::set_redu_ratio setRatioSrv;
    inspire_hand_modbus::set_pos setPosSrv;

    // set ID
    setIdSrv.request.id = 1; // example value
    if (setIdClient.call(setIdSrv)) {
        ROS_INFO("Set ID successful: %d", setIdSrv.response.success);
    } else {
        ROS_ERROR("Failed to call service set_id");
    }

    // set reduction ratio
    setRatioSrv.request.redu_ratio = 10; // example value
    if (setRatioClient.call(setRatioSrv)) {
        ROS_INFO("Set reduction ratio successful: %d", setRatioSrv.response.success);
    } else {
        ROS_ERROR("Failed to call service set_redu_ratio");
    }

    // set position
    setPosSrv.request.pos0 = 300; // pinky
    setPosSrv.request.pos1 = 300; // ring
    setPosSrv.request.pos2 = 300; // middle
    setPosSrv.request.pos3 = 300; // index
    setPosSrv.request.pos4 = 1000; // thumb flexion
    setPosSrv.request.pos5 = 1000; // thumb abduction
    if (setPosClient.call(setPosSrv)) {
        ROS_INFO("Set position successful: %d", setPosSrv.response.success);
    } else {
        ROS_ERROR("Failed to call service set_pos");
    }

    // send Modbus request, pass the position parameters
    sendModbusPositionCommand(
        setPosSrv.request.pos0, 
        setPosSrv.request.pos1, 
        setPosSrv.request.pos2, 
        setPosSrv.request.pos3, 
        setPosSrv.request.pos4, 
        setPosSrv.request.pos5
    );

    // close the Modbus connection
    modbus_close(ctx);
    modbus_free(ctx);

    return 0;
}

