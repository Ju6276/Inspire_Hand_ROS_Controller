#include <ros/ros.h>
#include "hand_control.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "hand_modbus_control");  // node name
    ros::NodeHandle nh;  // create the node handle

    inspire_hand::hand_serial handControl(&nh);
    
    // register the service
    ros::ServiceServer getErrorService = nh.advertiseService("inspire_hand_modbus/get_error", 
        &inspire_hand::hand_serial::getERRORCallback, &handControl);
    
    ros::ServiceServer getForceActService = nh.advertiseService("inspire_hand_modbus/get_force_act", 
        &inspire_hand::hand_serial::getFORCE_ACTCallback, &handControl);
        
    ros::ServiceServer getCurrentService = nh.advertiseService("inspire_hand_modbus/get_current", 
        &inspire_hand::hand_serial::getCURRENTCallback, &handControl);
        
    ros::ServiceServer getAngleSetService = nh.advertiseService("inspire_hand_modbus/get_angle_set", 
        &inspire_hand::hand_serial::getANGLE_SETCallback, &handControl);
        
    ros::ServiceServer getAngleActService = nh.advertiseService("inspire_hand_modbus/get_angle_act", 
        &inspire_hand::hand_serial::getANGLE_ACTCallback, &handControl);
        
    ros::ServiceServer getForceSetService = nh.advertiseService("inspire_hand_modbus/get_force_set", 
        &inspire_hand::hand_serial::getFORCE_SETCallback, &handControl);
    
    ros::ServiceServer getTempService = nh.advertiseService("inspire_hand_modbus/get_temp", 
        &inspire_hand::hand_serial::getTEMPCallback, &handControl);
    
    ros::ServiceServer getPosSetService = nh.advertiseService("inspire_hand_modbus/get_pos_set", 
        &inspire_hand::hand_serial::getPOS_SETCallback, &handControl);
    
    ros::ServiceServer getPosActService = nh.advertiseService("inspire_hand_modbus/get_pos_act", 
        &inspire_hand::hand_serial::getPOS_ACTCallback, &handControl);   
    
    ros::ServiceServer getStatusService = nh.advertiseService("inspire_hand_modbus/get_status", 
        &inspire_hand::hand_serial::getSTATUSCallback, &handControl);
    
    ros::ServiceServer setClearErrorService = nh.advertiseService("inspire_hand_modbus/set_clear_error", 
        &inspire_hand::hand_serial::setCLEAR_ERRORCallback, &handControl);

    ros::ServiceServer setIdService = nh.advertiseService("inspire_hand_modbus/set_id", 
        &inspire_hand::hand_serial::setIDCallback, &handControl);
        
    ros::ServiceServer setRatioService = nh.advertiseService("inspire_hand_modbus/set_redu_ratio", 
        &inspire_hand::hand_serial::setREDU_RATIOCallback, &handControl);
        
    ros::ServiceServer setPosService = nh.advertiseService("inspire_hand_modbus/set_pos", 
        &inspire_hand::hand_serial::setPOSCallback, &handControl);
        
    ros::ServiceServer setGestureNoService = nh.advertiseService("inspire_hand_modbus/set_gesture_no", 
        &inspire_hand::hand_serial::setGESTURE_NOCallback, &handControl);
        
    ros::ServiceServer setSpeedService = nh.advertiseService("inspire_hand_modbus/set_speed", 
        &inspire_hand::hand_serial::setSPEEDCallback, &handControl);
    
    ros::ServiceServer setDefaultSpeedService = nh.advertiseService("inspire_hand_modbus/set_default_speed", 
        &inspire_hand::hand_serial::setDEFAULT_SPEEDCallback, &handControl);
        
    ros::ServiceServer setAngleService = nh.advertiseService("inspire_hand_modbus/set_angle", 
        &inspire_hand::hand_serial::setANGLECallback, &handControl);
        
    ros::ServiceServer setSaveFlashService = nh.advertiseService("inspire_hand_modbus/set_save_flash", 
        &inspire_hand::hand_serial::setSAVE_FLASHCallback, &handControl);
        
    ros::ServiceServer setForceService = nh.advertiseService("inspire_hand_modbus/set_force", 
        &inspire_hand::hand_serial::setFORCECallback, &handControl);
        
    ros::ServiceServer setForceClbrService = nh.advertiseService("inspire_hand_modbus/set_force_clb", 
        &inspire_hand::hand_serial::setFORCE_CLBCallback, &handControl);
        
    ros::ServiceServer setDefaultForceService = nh.advertiseService("inspire_hand_modbus/set_default_force", 
        &inspire_hand::hand_serial::setDEFAULT_FORCECallback, &handControl);
        
    ros::ServiceServer setCurrentLimitService = nh.advertiseService("inspire_hand_modbus/set_current_limit", 
        &inspire_hand::hand_serial::setCURRENT_LIMITCallback, &handControl);
        
    ros::ServiceServer setResetParaService = nh.advertiseService("inspire_hand_modbus/set_reset_para", 
        &inspire_hand::hand_serial::setRESET_PARACallback, &handControl);    
        
    ROS_INFO("Hand control service ready."); 
       
    inspire_hand::hand_serial hand(&nh);

    // read the address of the register (optional)
    int reg_addr = 2016; // replace with the address of the register to read
    int value = hand.getRegisterValue(reg_addr);

    if (value != -1) { // check if the read is successful
        ROS_INFO("Successfully read register %d: value = %d", reg_addr, value);
    } else {
        ROS_ERROR("Failed to read register %d", reg_addr);
    }

    ros::spin();  // enter the ROS event loop
    return 0;
}

