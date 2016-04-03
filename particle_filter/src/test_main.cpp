#include <ros/ros.h> 
#include <particle_filter/ParticleFilter.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <Eigen/Eigen>
#include <sensor_msgs/PointCloud.h>
#include <particle_filter/catheter_msg.h>
#include <std_msgs/Float32.h>
using namespace std;
sensor_msgs::PointCloud getSamplePoints(std::vector<CatheterPose> cps);
double compareSamples(CatheterPose motion_c, CatheterPose msr_c);

CatheterPose g_mcp;
bool g_gotCP = false;
void Callback(const particle_filter::catheter_msg& mcp)
{
    ROS_INFO("got measurement data.");
    g_mcp.A_ = mcp.A;
    g_mcp.B_ = mcp.B;
    g_mcp.C_ = mcp.C;
    g_gotCP = true; 
}

double g_amps;
bool g_gotamps=false;
void ampsCallback(const std_msgs::Float32& amps)
{
    g_amps = amps.data;
    g_gotamps = true;
    ROS_INFO("Got input %f amps.", g_amps);
}

int main(int argc, char **argv) 
{ 
    ros::init(argc,argv,"test_main"); 
    ros::NodeHandle n;
    ros::Subscriber catheter_msrd_sub= n.subscribe("measured_catheter",1,Callback); 
    ros::Publisher catheter_pts_pub = n.advertise<sensor_msgs::PointCloud>("catheter_sample_pts_rviz", 1);
    ros::Subscriber amps_sub = n.subscribe("amps",1,ampsCallback); 
    int nm=500;
    ParticleFilter pf(nm);
    geometry_msgs::Point v_B;
    double v_a; 
    v_B.x=0.002;
    v_B.y=0.003;
    v_B.z=0.0015;
    v_a = 0.003;

/* start from here */
    pf.setMotionErrors(v_B,v_a); // set the standard variances for motion model

    // set current and generate particles
    while(!g_gotamps)
    {
        ros::spinOnce();
        ROS_INFO("wait for amps input...");
        ros::Duration(1).sleep();
    }
    pf.generateParticles(g_amps);

    std::vector<CatheterPose> cp=pf.getParticles();
    sensor_msgs::PointCloud sample_pts;
    std::vector<double> vr;
    vr.resize(2);
    while(ros::ok())
    {
        // display all the samples
        cp.clear();
        cp.resize(nm);
        cp=pf.getParticles();
        sample_pts = getSamplePoints(cp);
        catheter_pts_pub.publish(sample_pts);
        while(!g_gotCP)
        {
            ros::spinOnce();
            ROS_INFO("wait for measurement data...");
            ros::Duration(1).sleep();
        };
        g_gotCP = false;

        /* Set the weights*/
        std::vector<double> w;
        w.clear();
        for (int j=0;j<nm;j++)
        {
            w.push_back(compareSamples(cp[j],g_mcp));
        }
        bool t=pf.setWeights(w);

        /* Resample*/
        //pf.resample(); // resample without reducing the number of particles
        ROS_INFO("Resampling...");
        nm=pf.resample2(); // this one will reduce the number of particles, and returns the number of new particles
        cout<<"Current samples number: "<<nm<<endl;
        vr = pf.getVar();
        cout<<"Current sample variances: B: "<<vr[0]<<", alpha: "<<vr[1]<<endl;
        ros::Duration(1).sleep();

        if (vr[0]<0.0015 && vr[1] < 0.0028 || nm <10)
          break;
    } 
    double best_weight;
    CatheterPose mean_cp = pf.getMeanParticle();
    CatheterPose best_cp = pf.getBestParticle(best_weight);
    cout<<"The mean catheter:"<<endl;
    mean_cp.displayCatheter();
    cout<<"The best catheter:"<<endl;
    best_cp.displayCatheter(); 

    return 0; 
} 

sensor_msgs::PointCloud getSamplePoints(std::vector<CatheterPose> cps)
{
    int pts_nm = cps.size();
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "catheter_base";
    cloud.points.resize(3*pts_nm);

    int j=0;
    for (int i=0; i<pts_nm; i++)
    {
        cloud.points[j].x = cps[i].A_.x*10;
        cloud.points[j].y = cps[i].A_.y*10;
        cloud.points[j].z = cps[i].A_.z*10;

        j++;
        cloud.points[j].x = cps[i].B_.x*10;
        cloud.points[j].y = cps[i].B_.y*10;
        cloud.points[j].z = cps[i].B_.z*10;

        j++;
        cloud.points[j].x = cps[i].C_.x*10;
        cloud.points[j].y = cps[i].C_.y*10;
        cloud.points[j].z = cps[i].C_.z*10;

        j++;
    }

    return cloud;
}


double compareSamples(CatheterPose motion_c, CatheterPose msr_c)
{
    double weight=0;
    geometry_msgs::Point msr_p;
    geometry_msgs::Point motion_p;
    double x,y,z;
    msr_p = msr_c.A_;
    motion_p = motion_c.A_;
    x = msr_p.x - motion_p.x;
    y = msr_p.y - motion_p.y;
    z = msr_p.z - motion_p.z;
    weight += sqrt(x*x+y*y+z*z);

    msr_p = msr_c.B_;
    motion_p = motion_c.B_;
    x = msr_p.x - motion_p.x;
    y = msr_p.y - motion_p.y;
    z = msr_p.z - motion_p.z;
    weight += sqrt(x*x+y*y+z*z);

    msr_p = msr_c.C_;
    motion_p = motion_c.C_;
    x = msr_p.x - motion_p.x;
    y = msr_p.y - motion_p.y;
    z = msr_p.z - motion_p.z;
    weight += sqrt(x*x+y*y+z*z);

    weight = 1.0/weight;
    return weight;
}
