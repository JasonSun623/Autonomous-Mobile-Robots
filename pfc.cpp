#include "point.h"
#include <math.h>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point32.h"
#include "nav_msgs/Odometry.h"
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "geometry_msgs/PoseWithCovariance.h"

using namespace std;
using namespace ros;

void handle_odom( const geometry_msgs::PoseWithCovariance::ConstPtr& msg);
Point lookAheadPoint(Point mu, vector<float> path_x, vector<float> path_y);
float distanceP(Point & A, Point & B);  // distance is in the std namespace
Point findClosestPoint(Point A, Point B, Point mu);
void handle_path(const geometry_msgs::Polygon::ConstPtr& msg);
float runPurePursuit();

float Wthresh = 0.1;
float maxV_w = M_PI;
float lookAheadDistance = 0.17;
float V_l = 0.1;
float linVel;
float V_w;
Point lookahead(0,0);
Point Mu(0,0);
float quaternion[4] = {0,0,0,1};
float discResolution = 30.0;
float lookAheadThresh = 0.05;
int PathLength = 1;

vector<float> oldPath_x;
vector<float> path_x;
vector<float> path_y;
vector<int> passed;

int main(int argc, char** argv) {
    init(argc, argv, "pfc");
    geometry_msgs::Twist cmd;
    geometry_msgs::Point32 p;
    geometry_msgs::Polygon path_and_lookahead;
    geometry_msgs::Polygon next_path;

    NodeHandle n;
	Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);
    Publisher pathPub = n.advertise<geometry_msgs::Polygon>("path_and_lookahead", 1000);
    Subscriber sub = n.subscribe("/pos",1000, handle_odom);
    Subscriber np = n.subscribe("DistPath",100, handle_path);
	ros::Rate loop_rate(10);

    while (ros::ok()) {
        p.x = Mu.x;
        p.y = Mu.y;
        path_and_lookahead.points.push_back(p);
        if (path_x.size() != 0) {
            Point waypoint(path_x[PathLength], path_y[PathLength]);
            if (distanceP(Mu, waypoint) < Wthresh) {
                PathLength++;
            }
            V_w = runPurePursuit();
            linVel = V_l;
            path_and_lookahead.points.clear();
            p.x = lookahead.x;
            p.y = lookahead.y;
            p.z = 0;
            path_and_lookahead.points.push_back(p);
            for (int g = 0; g < path_x.size(); g++) {
                p.x = path_x[g];
                p.y = path_y[g];
                p.z = 0;
                path_and_lookahead.points.push_back(p);
            }
        }
        if (path_x.size() != 0) {
            Point Goal(path_x[path_x.size()-1], path_y[path_y.size()-1]);
            if (distanceP(Mu, Goal) < Wthresh) {
                linVel = 0;
                V_w = 0;
            }
        }
        if (linVel > V_l) {linVel = V_l;}
	cout << "The command is: " << linVel << "  " << V_w << endl;
        cmd.linear.x = linVel;
        cmd.angular.z = V_w;
        ros::spinOnce();
		pub.publish(cmd);
        pathPub.publish(path_and_lookahead);
		loop_rate.sleep();
	}
}

float runPurePursuit() {
    float angVelocity;
    tf::Quaternion q(quaternion[1], quaternion[2], quaternion[3], quaternion[0]);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    lookahead = lookAheadPoint(Mu, path_x, path_y);

    float y_offset = (lookahead.x-Mu.x)*sin(-yaw)+(lookahead.y-Mu.y)*cos(-yaw);

    float lambda = (2*(y_offset))/pow(lookAheadDistance,2);
    angVelocity = V_l*lambda;
    if (angVelocity > maxV_w) {
        angVelocity = maxV_w;
    }
    return angVelocity;
}

void handle_path(const geometry_msgs::Polygon::ConstPtr& msg) {
        path_x.clear();
        path_y.clear();
        for (int i = 0; i < msg->points.size(); i++) {
            path_x.push_back(msg->points[i].x);
            path_y.push_back(msg->points[i].y);
        }
}

// listen to odometry for tuning
void handle_odom( const geometry_msgs::PoseWithCovariance::ConstPtr& msg) {
    Mu.x = msg->pose.position.x;
    Mu.y = msg->pose.position.y;
    quaternion[0] = msg->pose.orientation.w;
    quaternion[1] = msg->pose.orientation.x;
    quaternion[2] = msg->pose.orientation.y;
    quaternion[3] = msg->pose.orientation.z;
}

Point lookAheadPoint(Point mu, vector<float> path_x, vector<float> path_y) {
    Point goal(path_x[path_x.size()-1], path_y[path_y.size()-1]);
    Point p(path_x[PathLength-1], path_y[PathLength-1]);
    Point n(path_x[PathLength], path_y[PathLength]);
    //cout << "P: " << p.x << " " << p.y << " N: " << n.x << " " << n.y << endl;
    Point closest = findClosestPoint(p, n, mu);
    if (distanceP(mu, goal) < lookAheadDistance ){
        return Point(path_x[path_x.size()-1], path_y[path_y.size()-1]);
    }

    if (distanceP(mu, closest) > lookAheadDistance) {
        return closest;
    }

    vector<Point> discPath;
    float xd, yd, xInc, yInc;
    Point start(0,0);
    for (int i = PathLength-1; i < path_x.size()-1; i++) {
        float nextWaypoint = i + 1; 
        if (i == PathLength-1) {
            start.setx(closest.x);
            start.sety(closest.y);
        }
        else {
            start.setx(path_x[i]);
            start.sety(path_y[i]);
        }
        xd = path_x[nextWaypoint] - start.x;
        yd = path_y[nextWaypoint] - start.y;
        xInc = xd/discResolution;
        yInc = yd/discResolution;
        for (int g = 0; g < (int)discResolution; g++) {
            discPath.push_back(Point(start.x+g*xInc, start.y+g*yInc));
        }
    }

    for (int h = 0; h < discPath.size(); h++) {
        if (abs(distanceP(closest, discPath[h]) - lookAheadDistance) < lookAheadThresh ) {
            return discPath[h];
        }
    }
}

float distanceP(Point & A, Point & B) {
    return sqrt(pow(A.x-B.x,2) + pow(A.y-B.y,2));
}


Point findClosestPoint(Point A, Point B, Point P) {
    if (A.x == B.x && A.y == B.y) {
        return Point(A.x, P.x);
    }
    vector<float> a_to_p;
    a_to_p.push_back(P.x - A.x);
    a_to_p.push_back(P.y - A.y);

    vector<float> a_to_b;
    a_to_b.push_back(B.x - A.x);
    a_to_b.push_back(B.y - A.y);

    float atb2 = pow(a_to_b[0],2)+pow(a_to_b[1],2);

    float atp_dot_atb = a_to_p[0]*a_to_b[0] + a_to_p[1]*a_to_b[1];

    float t = atp_dot_atb/atb2;

    Point closest(A.x+a_to_b[0]*t, A.y+a_to_b[1]*t);
    return closest;
}
