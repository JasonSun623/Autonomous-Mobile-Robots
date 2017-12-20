#include <iostream>
#include <ros/ros.h>
#include "point.h"
#include "geometry_msgs/Polygon.h"
#include "actionlib_msgs/GoalStatus.h"
#include "geometry_msgs/Point.h"


using namespace std;
using namespace ros;

void path_goalStatus(const actionlib_msgs::GoalStatus::ConstPtr& msg);

int flag = 0;
int halt = 0;

int main(int argc, char** argv) {
    init(argc, argv, "exec");

    geometry_msgs::Point goal;
    actionlib_msgs::GoalStatus h;

    NodeHandle n;
    Publisher npath = n.advertise<geometry_msgs::Point>("next_goal", 1000);
    Publisher reqPath = n.advertise<actionlib_msgs::GoalStatus>("plannerFlag",1000, true);
    Subscriber reqGoal = n.subscribe("next_pathFlag", 1000, path_goalStatus);
	ros::Rate loop_rate(10);

   
    vector<Point> goals;
    goals.push_back(Point(0.5, 0.0));
    goals.push_back(Point(1.5, 0.0));
    goals.push_back(Point(2.5, 0.0));
    goals.push_back(Point(4.0, 0.0));
    goals.push_back(Point(5.5, 0.0));
    goals.push_back(Point(6.5, 0.0));
    goals.push_back(Point(7.5, 0.0));
    goals.push_back(Point(6.5, 0.0));
    goals.push_back(Point(5.5, 0.0));
    goals.push_back(Point(4.0, 0.0));
    goals.push_back(Point(2.5, 0.0));
    goals.push_back(Point(1.5, 0.0));
    while (ros::ok()) {
        h.status = 1;
        if (flag && !goals.empty()) {
            cout << "Goal : " << endl;
            goal.x = goals.back().x;
            goal.y = goals.back().y;
            cout << "Goal x-y: " << goal.x << " " << goal.y << endl;
            goals.pop_back();
            h.status = 3;
            flag = 0;
        }
        ros::spinOnce();
        npath.publish(goal);
        reqPath.publish(h);
		loop_rate.sleep();
    }

}

void path_goalStatus(const actionlib_msgs::GoalStatus::ConstPtr& msg) {
    if (msg->status == 3) {
        cout << "Goal status" << endl;
        flag = 1;
    }
}
