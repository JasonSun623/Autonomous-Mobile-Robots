#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <QtGui/QApplication>
#include <QtGui/QWidget>
#include <QtOpenGL/QGLWidget>
#include <QtCore>
#include <QTimer>
#include <GL/glu.h>
#include <vector>
#include "point.h"
#include "geometry_msgs/Polygon.h"

class GUI: public QGLWidget {
    Q_OBJECT
    float posx, posy;
    float robot_x, robot_y;
    std::vector<float> scans;
    float angleMin, angleIncrement;
    std::vector<float> quaternion;
    Point lookahead;
    std::vector<Point> path;
    public:
        GUI( QWidget * parent = NULL );
        virtual ~GUI();
        void handle_laserscan( const sensor_msgs::LaserScan::ConstPtr& msg );
        void handle_odom( const nav_msgs::Odometry::ConstPtr& msg );
        void handle_robot_mu( const geometry_msgs::PoseWithCovariance::ConstPtr& msg);
        void handle_path(const geometry_msgs::Polygon::ConstPtr& msg);
        QTimer* timer;
        protected slots:
        void timer_callback( void );
    protected:
        virtual void initializeGL();
        virtual void paintGL();
};
