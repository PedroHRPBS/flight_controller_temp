#include "RestrictedNormWaypointRefGenerator.hpp"

RestrictedNormWaypointRefGenerator::RestrictedNormWaypointRefGenerator(){
    _input_port_0 = new InputPort(ports_id::IP_0_WAYPOINT, this);
    _input_port_1 = new InputPort(ports_id::IP_1_SETTINGS, this);
    _input_port_2 = new InputPort(ports_id::IP_2_DATA, this);
    _output_port_0 = new OutputPort(ports_id::OP_0_X, this);
    _output_port_1 = new OutputPort(ports_id::OP_1_Y, this);
    _output_port_2 = new OutputPort(ports_id::OP_2_Z, this);
    _output_port_3 = new OutputPort(ports_id::OP_3_YAW, this);
    _output_port_4 = new OutputPort(ports_id::OP_4_COUNTER, this);

    _ports = {_input_port_0, _input_port_1, _input_port_2, _output_port_0, _output_port_1, _output_port_2, _output_port_3, _output_port_4};
}

void RestrictedNormWaypointRefGenerator::process(DataMessage* t_msg, Port* t_port){
    if (t_port->getID() == ports_id::IP_0_WAYPOINT){
        PosesMsg* t_pos_msg=(PosesMsg*) t_msg;
   
        for(int i=0;i<t_pos_msg->p.poses.size();i++){
            Waypoint t_waypoint;
            t_waypoint.position.x=t_pos_msg->p.poses[i].x;
            t_waypoint.position.y=t_pos_msg->p.poses[i].y;
            t_waypoint.position.z=t_pos_msg->p.poses[i].z;
            t_waypoint.yaw=t_pos_msg->p.poses[i].yaw;
            std::cout << "waypoint X : " << t_waypoint.position.x << std::endl;
            std::cout << "waypoint Y : " << t_waypoint.position.y << std::endl;
            std::cout << "waypoint Z : " << t_waypoint.position.z << std::endl;
            std::cout << "waypoint Yaw : " << t_waypoint.yaw << std::endl;
            Waypoints.push_back(t_waypoint);
        }
    }
    else if(t_port->getID() == ports_id::IP_1_SETTINGS){
        RestrictedNormRefSettingsMsg* t_settings_msg=(RestrictedNormRefSettingsMsg*) t_msg;
        max_norm=t_settings_msg->getMaxNorm();
        enabled=t_settings_msg->enabled;
        std::cout << "ENABLED: " << enabled << std::endl;
        std::cout << "DELETE : " << t_settings_msg->delete_existing_waypoints << std::endl;
        std::cout << "MAX NORM : " << max_norm << std::endl;
        if (t_settings_msg->delete_existing_waypoints){
            Waypoints.clear();
        }
    }
    else if(t_port->getID() == ports_id::IP_2_DATA)
    {
        Vector3DMessage* t_current_pos=(Vector3DMessage*) t_msg;
        Vector3D<double> t_current_pos_vec;
        t_current_pos_vec = t_current_pos->getData();
      
        if (Waypoints.size()>0 && enabled){
            Vector3D<double> diff_pos_waypoint=Waypoints[0].position-t_current_pos_vec;
            double t_dist= Vector3D<double>::getL2Norm(diff_pos_waypoint);
            double dist_x = Waypoints[0].position.x - t_current_pos_vec.x;
            double dist_y = Waypoints[0].position.y - t_current_pos_vec.y;
            double dist_z = Waypoints[0].position.z - t_current_pos_vec.z;
            Vector3D<double> restricted_ref;
            restricted_ref.x = Waypoints[0].position.x;
            restricted_ref.y = Waypoints[0].position.y;
            restricted_ref.z = Waypoints[0].position.z;
            bool move_x=false, move_y=false, move_z=false;
            if (abs(dist_x)>=max_norm){
                restricted_ref.x=t_current_pos_vec.x + (dist_x/abs(dist_x))*max_norm;
                move_x = true;
            }
            if (abs(dist_y)>=max_norm){
                restricted_ref.y=t_current_pos_vec.y + (dist_y/abs(dist_y))*max_norm;
                move_y = true;
            }
            if (abs(dist_z)>=max_norm){
                restricted_ref.z=t_current_pos_vec.z + (dist_z/abs(dist_z))*max_norm;
                move_z = true;
            }
            
            if(move_x || move_y || move_z){
                updateControlSystemsReferences(restricted_ref,Waypoints[0].yaw);
            }else{
                if (Waypoints.size()==1){
                    updateControlSystemsReferences(Waypoints[0].position,Waypoints[0].yaw);
                    Waypoints.clear();
                }else{
                    if(t_dist <= 0.1){
                        auto firstWaypoint=Waypoints.begin();
                        Waypoints.erase(firstWaypoint);
                    }
                }
            }
        }

        FloatMsg num_waypoints;
        num_waypoints.data = Waypoints.size();
        this->_output_port_4->receiveMsgData(&num_waypoints);

    }
}

std::vector<Port*> RestrictedNormWaypointRefGenerator::getPorts(){
    return this->_ports;
}

DataMessage* RestrictedNormWaypointRefGenerator::runTask(DataMessage*){

}

void RestrictedNormWaypointRefGenerator::receiveMsgData(DataMessage* t_msg){
    
}

void RestrictedNormWaypointRefGenerator::receiveMsgData(DataMessage* t_msg, int t_channel){
    
}


void RestrictedNormWaypointRefGenerator::updateControlSystemsReferences(Vector3D<double> t_pos_ref, double t_yaw){
    
    FloatMsg x_cont_ref;
    x_cont_ref.data = t_pos_ref.x;
    this->_output_port_0->receiveMsgData(&x_cont_ref);

    FloatMsg y_cont_ref;
    y_cont_ref.data = t_pos_ref.y;
    this->_output_port_1->receiveMsgData(&y_cont_ref);

    FloatMsg z_cont_ref;
    z_cont_ref.data = t_pos_ref.z;
    this->_output_port_2->receiveMsgData(&z_cont_ref);

    FloatMsg yaw_cont_ref;
    yaw_cont_ref.data = t_yaw;
    this->_output_port_3->receiveMsgData(&yaw_cont_ref);

}
