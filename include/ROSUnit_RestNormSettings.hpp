#pragma once
#include "common_srv/ROSUnit.hpp"
#include "RestrictedNormRefSettingsMsg.hpp"
#include <flight_controller/Restricted_Norm_Settings.h>
#include "common_srv/Vector3D.hpp"
#include "OutputPort.hpp"

class ROSUnit_RestNormSettings :  public ROSUnit{

    private:
        static ROSUnit_RestNormSettings* _instance_ptr;
        static RestrictedNormRefSettingsMsg _settings_msg; 
        ros::ServiceServer _srv_rest_norm_settings;
        static bool callbackSettings(flight_controller::Restricted_Norm_Settings::Request  &req, 
                                     flight_controller::Restricted_Norm_Settings::Response &res);
        Port* _output_port;
        std::vector<Port*> _ports;

    public:
        enum ports_id {OP_0_DATA};
        void process(DataMessage* t_msg, Port* t_port);
        std::vector<Port*> getPorts();
        DataMessage* runTask(DataMessage*);

        void receiveMsgData(DataMessage* t_msg);  
        ROSUnit_RestNormSettings(ros::NodeHandle&);
        ~ROSUnit_RestNormSettings();
};