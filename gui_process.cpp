#include <QtGui/QApplication>
#include "gui.h"
#include "gui_process_cmdline.h"

using namespace std;

int main( int argc, char* argv[] ){
    gengetopt_args_info args;
    cmdline_parser( argc, argv, &args );
    QApplication app( argc, argv );
    ros::init( argc, argv, "gui" );
    ros::NodeHandle node_handle;
    GUI gui;
    ros::Subscriber subscriber_reset_odometry = node_handle.subscribe( "/scan", 1, &GUI::handle_laserscan, &gui );
    ros::Subscriber subscriber_odom = node_handle.subscribe( "/odom", 1, &GUI::handle_odom, &gui );
    ros::Subscriber subscriber_robot_pose = node_handle.subscribe("/pos", 1, &GUI::handle_robot_mu, &gui);
    ros::Subscriber path_sub = node_handle.subscribe("path_and_lookahead", 1, &GUI::handle_path, &gui);
    gui.show();
    return app.exec();
}
