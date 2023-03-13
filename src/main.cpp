#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>


#define k0 1.00000359 // scale factor k0
#define a 6378137 // semi-major axis a
#define b 6356752 // semi-minor axis b
#define f 1/298.257223563 // Flattening f
#define e 0.00669438 // eccentricity e

using namespace Eigen;
using namespace std;

static ros::Publisher pose_pub;

double calcM(double phi){
    auto res = a*((1 - pow(e,2)/4 - 3*pow(e,4)/64 - 4*pow(e,6)/256)*phi
                - (3*pow(e,2)/8 + 3*pow(e,4)/32 +45*pow(e,6)/1024)*pow(sin(phi),2)
                + (15*pow(e,4)/256 + 45*pow(e,6)/1024)*pow(sin(phi),4)
                - (35*pow(e,6)/3072)*pow(sin(phi),6));
    return res;
}
Vector3d latlon2utm(double lat, double lon, int zone){

    auto phi = lat;
    auto phi0 = 38 * M_PI/180; // the origin of coordinate of korea (38,127.5)

    auto lambda = lon;
    auto lambda0 = 127.5 * M_PI/180;
    

    static double eds = pow(e,2)/(1-pow(e,2)); // e dot squares
    auto N = a/sqrt(1-pow(e,2)*pow(sin(phi),2));
    auto T = pow(tan(phi),2);
    auto C = eds*pow(cos(phi),2);
    auto A = (lambda - lambda0)*cos(phi);
    auto M = calcM(phi);
    static double M0 = calcM(phi0);

    Vector3d res = Vector3d::Zero();

    res[0] = k0*N*(A + (1-T+C)*pow(A,3)/6 + (5 - 18*T + pow(T,2) + 72*C - 58*(pow(eds,2)))*pow(A,5)/120);
    res[1] = k0*(M - M0 + N*tan(phi)*(pow(A,2)/2 + (5 - T + 9*C + 4*pow(C,2))*pow(A,4)/24 + (61 - 58*T + pow(T,2) + 600*C - 330*pow(eds,2))*pow(A,6)/720));
    auto k = k0*(1 + (1-C)*pow(A,2)/2 + (5-4*T+42*C+13*pow(C,2)-28*pow(eds,2))*pow(A,4)/24 + (61 - 148*T + 16*pow(T,2))*pow(A,6)/720);

    return res;
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg){
    static bool gps_init = true;
    static Vector3d pose_init(0,0,0);
    geometry_msgs::PoseStamped pose;
    auto lat = gps_msg->latitude * M_PI/180.;
    auto lon = gps_msg->longitude * M_PI/180.;
    int zone;
    if (lon < 0){
        zone = (lon + 180)/6 + 1;
    }
    else{
        zone = lon/6 +31;
    }
    
    if(gps_init){
        pose_init = latlon2utm(lat,lon,zone);
        gps_init = false;
    }
    else{
        auto pose_tmp = latlon2utm(lat,lon,zone) - pose_init;
        pose.pose.position.x = pose_tmp[0];
        pose.pose.position.y = pose_tmp[1];
        
        pose_pub.publish(pose);
        std::cout << "x: " << pose.pose.position.x << endl;
        std::cout << "y: " << pose.pose.position.y << endl;
    }
    

    
}

int main(int argc, char** argv){
    ros::init(argc, argv, "gps2xy_node");
    ros::NodeHandle nh;
    ros::Subscriber gps_sub;
    gps_sub = nh.subscribe("/fix", 1, gps_callback);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/usv_pose",1);

    ros::spin();
    return 0;
}