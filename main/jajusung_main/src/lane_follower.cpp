#include "EKFH.h"
#include "tracker.h"
#include <iostream>

Lane_Controller::Lane_Controller(EKF* input, ros::NodeHandle &nh) {
    nh.getParam("ekf_mode", ekf_mode);
    nh.getParam("lane_stanley_k", LANE_K);
    nh.getParam("velocity_profile", VEL);
    lane_ekf = input;
    lane_ekf->dt = 0.037; lane_ekf->k = LANE_K; lane_ekf->wb = WB;

    cmd_pub = nh.advertise<erp42_msgs::DriveCmd>("drive", 10); 
    mod_pub = nh.advertise<erp42_msgs::ModeCmd>("mode", 10); 

    lane_sub = nh.subscribe("lane_result", 10, &Lane_Controller::lane_cb, this); 
}

int Lane_Controller::normalize_delta(int delta) {
    delta = delta>=28 ? 28 : delta;
    delta = delta<=-28 ? -28 : delta;
    return delta;
}

void Lane_Controller::lane_cb(const lane_ctrl::lane_info::ConstPtr& data) {
    this_left = double(data->left_x);
    this_right = double(data->right_x);
    std::cout<<"left, right : "<<this_left<<' '<<this_right<<'\n';
    left_theta = deg2rad(data->left_theta);
    right_theta = deg2rad(data->right_theta);
    stanley();
}

void Lane_Controller::stanley() {
    if(this_left == 200) {
        lat_err = (0.5-(this_right/400.0))*WIDTH;
        heading_err = PI/2 - right_theta;
    }
    else if(this_right == 200) {
        lat_err = -(0.5-(this_left/400.0))*WIDTH;
        heading_err = -(PI/2 - left_theta);
    }
    else{
        lat_err = ((this_left/(this_left+this_right))-0.5)*WIDTH;
        heading_err = PI/2 - (left_theta+right_theta)/2;
    }
    lane_ekf->z(0) = lat_err;
    lane_ekf->z(1) = heading_err;

    if(ekf_mode) {

        if(!drive_start) {
            // 주행 start시만 초기값 부여
            drive_start = true;
            lane_ekf->x(0) = lat_err;
            lane_ekf->x(1) = heading_err;

            stanley_delta = heading_err + atan(LANE_K*lat_err/((VEL/3.6)));
            lane_ekf->u(0) = VEL;
            lane_ekf->u(1) = stanley_delta;
        }
        
        lane_ekf->Filtering();
        lane_ekf->u(0) = VEL;
        lane_ekf->u(1) = lane_ekf->x(1) + atan(LANE_K*lane_ekf->x(0)/((VEL/3.6)));

        stanley_delta = rad2deg(lane_ekf->x(1) + atan(LANE_K*lane_ekf->x(0)/((VEL/3.6))));

    }

    // 최종 output은 degree
    else stanley_delta = rad2deg(heading_err + atan(LANE_K*lat_err/((VEL/3.6))));

    erp42_msgs::DriveCmd drive_cmd;
    erp42_msgs::ModeCmd mode_cmd;

    drive_cmd.KPH = VEL;
    drive_cmd.Deg = normalize_delta(stanley_delta);
    mode_cmd.MorA = 0x01;
    mode_cmd.EStop = 0x00;
    mode_cmd.Gear = 0x00;
    cmd_pub.publish(drive_cmd);
    mod_pub.publish(mode_cmd);
            
    // debug
    std::cout<<"======================="<<'\n';
    std::cout<<"heading err : "<<heading_err<<"\n";
    std::cout<<"lat_err : "<<lat_err<<"\n";
    std::cout<<"delta : "<<stanley_delta<<"\n";
    std::cout<<"predicted vector : "<<'\n'<<lane_ekf->x_p<<'\n';
    std::cout<<"state vector : "<<'\n'<<lane_ekf->x<<'\n';
    std::cout<<"observation vector : "<<'\n'<<lane_ekf->z<<'\n';

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Lane_Controller");
    ros::NodeHandle nh;
    EKF lane_ekf(2,2);
    Lane_Controller lane_ctrl(&lane_ekf, nh);
    ros::spin();
    return 0;
}