#include "Localizer.h"
#include "LocalizerStart_cmdline.h"
#include <ros/ros.h>
#include "geometry_msgs/PoseWithCovariance.h"
#include <Eigen/Dense>
#include "nav_msgs/Odometry.h"

using namespace std;
float updateFreq = 60;



int main(int argc, char* argv[]) {
    gengetopt_args_info args;
    cmdline_parser(argc,argv,&args);
    ros::init(argc, argv, "localizer");
    geometry_msgs::PoseWithCovariance msg;
    Localizer localizer;
    localizer.setConeRadii(args.coneRadii_arg);
    float alphas = 0.01;
    localizer.setAlpha(alphas);
    localizer.setUpdateRate(updateFreq);
    ros::NodeHandle n;
    ros::Publisher PosWithCovPub = n.advertise<geometry_msgs::PoseWithCovariance>("/pos", 1);
    ros::Subscriber cntrl = n.subscribe("/odom", 1000, &Localizer::cmdUpdate, &localizer);
	ros::Subscriber beams = n.subscribe("/scan", 1000, &Localizer::handleScans, &localizer);
    ros::Rate loop_rate(updateFreq);
    while (ros::ok()) {
        if (localizer.scans.size() != 0) {
            localizer.findFeature();
            localizer.EKF();
        }
        msg.pose.position.x = localizer.getx();
        msg.pose.position.y = localizer.gety();
        msg.pose.orientation.x = localizer.getQuatx();
        msg.pose.orientation.y = localizer.getQuaty();
        msg.pose.orientation.z = localizer.getQuatz();
        msg.pose.orientation.w = localizer.getQuatw();
        Eigen::Matrix3f Covariance = localizer.getSigma();
        int index = 0;
        for (int i = 0; i < Covariance.rows(); i++) {
            for (int k = 0; k < Covariance.cols(); k++) {
                msg.covariance[index] = Covariance(i,k);
                index++;
            }
        }
        ros::spinOnce();
        PosWithCovPub.publish(msg);
        loop_rate.sleep();
    }
    return 0;
}
