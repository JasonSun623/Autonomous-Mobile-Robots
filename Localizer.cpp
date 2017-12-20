#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <vector>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Twist.h"
#include "Localizer.h"
#include <Eigen/Dense>
#include "point.h"
#include "nav_msgs/Odometry.h"

using namespace std;

Point toGlobal(float range, float bearing, Eigen::RowVector3f mu);
float distanceP(Point & A, Point & B);

Localizer::Localizer() {
    Mx.resize(6);
    Mx[0] = 1.0;
    Mx[1] = 2.0;
    Mx[2] = 3.0;
    Mx[3] = 5.0;
    Mx[4] = 6.0;
    Mx[5] = 7.0;
    My.resize(6);
    My[0] = 0;
    My[1] = 0;
    My[2] = 0;
    My[3] = 0;
    My[4] = 0;
    My[5] = 0;


    Quter.resize(4);
    u << 0.00000000001, 0.00000000001;        // linear velocity, angular velocity
    mu << 0.0, 0.0, 0.0;  // x, y, theta of robot

    // distance, bearing, signature
    z.resize(6);
    for (int e = 0; e < 6; e++)
        z[e] << Mx[e], 0.0, e;
    St.resize(6);
    for (int k = 0; k < 6; k++) {
        St[k] << 0.1, 0.0, 0.0,    // covariance of beam returns - play with values
                 0.0, 0.1, 0.0,
                 0.0, 0.0, 0.1;
    }
    zest.resize(6);
    for (int k = 0; k < 6; k++)
        zest[k] << Mx[k], My[k], k;  // predicted beam return at initialization - range bearing signature
    Ht.resize(6);
    for (int k = 0; k < 6; k++) {
        Ht[k] << 0.0, 0.0, 0.0,
                 0.0, 0.0, -1.0,
                 0.0, 0.0, 0.0;
    }
    Kt.resize(6);
    for (int k = 0; k < 6; k++) {
        Kt[k] << 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0;
    }

    sigma << 0.0, 0.0, 0.0,     // covariance of robot pos
             0.0, 0.0, 0.0,
             0.0, 0.0, 0.0;
    Gt << 1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0;

    Vt.resize(3,2);
    Vt << 0.0, 0.0,
          0.0, 0.0,
          0.0, 0.0;

    Mt << 0.0, 0.0,
          0.0, 0.0;

    projMu << 0.0,
	          0.0,
	          0.0;
    projSigma << 0.0, 0.0, 0.0,
	     0.0, 0.0, 0.0,
	     0.0, 0.0, 0.0;
    Qt << 0.001, 0.0, 0.0,
          0.0, 0.001, 0.0,
          0.0, 0.0, 0.001;
}

void Localizer::setAlpha(float alphas) {
    alpha = alphas;
}

void Localizer::setUpdateRate(float freq) {
    dt = 1.0/freq;
}

void Localizer::EKF() {
    if (u(1) == 0.0) {
        u(1) = 0.0000001;
    }
    float theta = mu(2);

    float spr = u(0)/u(1);

    Gt(0,2) = (-spr*cos(theta))+(spr*cos(theta+u(1)*dt));
    Gt(1,2) = (-spr*sin(theta))+(spr*sin(theta+u(1)*dt));
    Vt(0,0) = (-sin(theta)+sin(theta+u(1)*dt))/u(1);
    Vt(1,0) = (cos(theta)-cos(theta+u(1)*dt))/u(1);
    Vt(0,1) = ( (u(0)*(sin(theta)-sin(theta+u(1)*dt)))/pow(u(1),2) ) + (u(0)*cos(theta+u(1)*dt)*dt)/u(1);
    Vt(1,1) = -( (u(0)*(cos(theta)-cos(theta+u(1)*dt)))/pow(u(1),2) ) + (u(0)*sin(theta+u(1)*dt)*dt)/u(1);
    Vt(2,1) = dt;
    Mt(0,0) = alpha*pow(u(0),2); + alpha*pow(u(1),2);
    Mt(1,1) = alpha*pow(u(0),2); + alpha*pow(u(1),2);
    projMu(0) = mu(0) + (-spr*sin(theta))+(spr*sin(theta+u(1)*dt));
    projMu(1) = mu(1) + (spr*cos(theta))-(spr*cos(theta+u(1)*dt));
    projMu(2) = mu(2) + u(1)*dt;
    if (projMu(2) > M_PI) {
        projMu(2) -= 2.0*M_PI;
    }
    else if (projMu(2) < -M_PI) {
        projMu(2) += 2.0*M_PI;
    }
    projSigma = Gt*sigma*Gt.transpose() + Vt*Mt*Vt.transpose();
    for (int i = 0; i < 6; i++) {
        float q = pow(Mx[i] - projMu(0), 2) + pow(My[i] - projMu(1), 2);
        zest[i](0) = sqrt(q);
        zest[i](1) = atan2(My[i] - projMu(1), Mx[i] - projMu(0)) - projMu(2);
        zest[i](2) = i;
        Ht[i](0,0) = -(Mx[i]-projMu(0))/sqrt(q);
        Ht[i](0,1) = -(My[i]-projMu(1))/sqrt(q);
        Ht[i](1,0) = (My[i]-projMu(1))/q;
        Ht[i](1,1) = -(Mx[i]-projMu(0))/q;
        St[i] = Ht[i]*projSigma*Ht[i].transpose() + Qt;
        Kt[i] = projSigma*Ht[i].transpose()*St[i].inverse();
        if(z[i](0) != -1000) projMu = projMu + Kt[i]*((z[i]-zest[i]).transpose());
        else projMu = projMu;
        Eigen::Matrix3f I;
        I << 1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0;
        projSigma = (I-Kt[i]*Ht[i])*projSigma;
    }

    Eigen::Matrix3f I;
    I << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;
    mu = projMu;
    sigma = projSigma;
    Quter[0] = cos(mu(2)/2);
    Quter[1] = 0;
    Quter[2] = 0;
    Quter[3] = sin(mu(2)/2);
}

void Localizer::handleScans(const sensor_msgs::LaserScan::ConstPtr& msg) {
    minAngle = msg->angle_min;
    maxRange = msg->range_max;
    angleIncrement = msg->angle_increment;
    scans.resize(msg->ranges.size());
    for (int i = 0; i < msg->ranges.size(); i++) {
        scans[i] = msg->ranges[i];
    }
}


void Localizer::cmdUpdate(const nav_msgs::Odometry::ConstPtr& msg) {
	u(0) = msg->twist.twist.linear.x;
	u(1) = msg->twist.twist.angular.z;
}


void Localizer::findFeature() {
    float beamAngle = minAngle;
    vector<Point> endbeams;
    float bx, by;
    //Point bend;
    for (int i = 0; i < scans.size(); i++) {
        if (scans[i] < 0.4 || scans[i] > 5.0) {beamAngle++; continue;}
        Point bend = toGlobal(scans[i], beamAngle, Localizer::mu);
       
        endbeams.push_back(bend);
        beamAngle += angleIncrement;
    }
    vector<vector<Point> > potentialBeams;
    potentialBeams.resize(6);
    //corr.resize(6);
    for (int o = 0; o < endbeams.size(); o++) {
        for (int g = 0; g < 6; g++) {
            Point cone(Mx[g],My[g]);
            if (distanceP(endbeams[o], cone) < 0.15) {
                cout << "Correspondance: " << g << endl;
                potentialBeams[g].push_back(endbeams[o]);
            }
        }
    }
    vector<float> mins;
    vector<Point> minsP;
    mins.resize(6,0);
    minsP.resize(6);
    for (int h = 0; h < 6; h++) {
        if (potentialBeams[h].size() > 5) {
            Point Mu(Localizer::mu(0), Localizer::mu(1));
            mins[h] = distanceP(Mu, potentialBeams[h][0]);
            minsP[h] = potentialBeams[h][0];
            for (int r = 1; r < potentialBeams[h].size(); r++) {
                if (distanceP(Mu, potentialBeams[h][r]) < mins[h]) {
                    mins[h] = distanceP(Mu, potentialBeams[h][r]);
                    minsP[h] = potentialBeams[h][r];
                }
            }
        }
    }
    cout << "Cone ";
    for (int f = 0; f < 6; f++) {
        if (mins[f] != 0) {
            Point Mu(Localizer::mu(0), Localizer::mu(1));
            float range = distanceP(minsP[f], Mu) + coneRadii;
            float bearing = atan2(minsP[f].y - Localizer::mu(1), minsP[f].x - Localizer::mu(0)) - Localizer::mu(2);
            //(minsP[f].x-Localizer::mu(0))*sin(Localizer::mu(2))+(minsP[f].y-Localizer::mu(1))*cos(Localizer::mu(2));
            z[f](0) = range;
            z[f](1) = bearing;
            z[f](2) = f;
            cout << " " << z[f](2) << " " << range;
        }
        else {
            z[f](0) = -1000;
            z[f](1) = -1000;
            z[f](2) = f;
        }
      
    }
    cout << endl;


}

Point toGlobal(float range, float bearing, Eigen::RowVector3f mu) {
    float Lx = range*cos(bearing);
    float Ly = range*sin(bearing);
    float Gx = Lx*cos(mu(2))-Ly*sin(mu(2))+mu(0);
    float Gy = Lx*sin(mu(2))+Ly*cos(mu(2))+mu(1);
    //cout << Gx << " " << Gy << endl;
    Point G(Gx, Gy);
    return G;
}


float distanceP(Point & A, Point & B) {
    return sqrt(pow(A.x-B.x,2) + pow(A.y-B.y,2));
}

void Localizer::setConeRadii(double r) {
    coneRadii = r;
}

float Localizer::getx() {
    return mu(0);
}

float Localizer::gety() {
    return mu(1);
}

float Localizer::getQuatx() {
    return Quter[1];
}

float Localizer::getQuaty() {
    return Quter[2];
}

float Localizer::getQuatz() {
    return Quter[3];
}

float Localizer::getQuatw() {
    return Quter[0];
}

Eigen::Matrix3f Localizer::getSigma() {
    return sigma;
}
