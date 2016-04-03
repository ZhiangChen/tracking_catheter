#ifndef FARTICLEFILTER_
#define FARTICLEFILTER_

#include <geometry_msgs/Point.h>
#include <Eigen/Eigen>
#include <ros/ros.h> 

/* parameters for Bezier spline */
const double beta1 = 0.5;
const double beta2 = 0.5;


/* parameters for motion model (kg.m.s)*/
const Eigen::Vector3d B(0,3,0); // magnetic filed
const double N=130; // the number of the coils
const double A=0.0000159; // the cross area of the coil
const double l_AB=0.08; // the length of the AB
const double l_BC=0.02; // the length of the BC 
const double m=0.0044;  // the mass of the coil set
const double E=101.5; // Young's modulus
const double I=0.00000427; // moment of inertia wrt z axis



struct CatheterPose
{
	geometry_msgs::Point A_;
	geometry_msgs::Point B_;
	geometry_msgs::Point C_;
	geometry_msgs::Point t_;
	double alpha_;
	geometry_msgs::Point getBSpline(double s);
	void displayCatheter();
	CatheterPose operator + (CatheterPose &c2); 
	CatheterPose operator / (double n);
	std::vector<double> operator - (CatheterPose &c2);
};

class ParticleFilter	
{
public:
	ParticleFilter(int n);
	ParticleFilter(double min_c, double max_c, double step, int n);
	void setMotionErrors(geometry_msgs::Point vars_B, double var_alpha); // set the standard variances for motion model
	std::vector<CatheterPose> getParticles(){return Catheters_;};
	bool setWeights(std::vector<double> w);
	bool generateParticles(double i);
	bool generateParticles();
	void icr_generateParticles(double d_i);

	void resample(); // resample without reducing the number of particles, return the number of valid samples
	int resample2(); // resample with reducing the number of particles, return the number of new samples
	CatheterPose getMeanParticle();
	CatheterPose getBestParticle(double &weight);
	CatheterPose getBestParticle();
	std::vector<double> getVar(); // return the standard variances of A, B, C and t;
	void displayCatheter(double index);

	
private:
	std::vector<CatheterPose> Catheters_;
	int nm_; // the number of particles
	double i_;// current in the coil
	double step_i_;
	double min_i_;
	int n_; // the number of particles for one current
	bool overflow_;
	bool i_state_; // true: generate particles based on only one current; false: on a range of currents  
	std::vector<double> weights_; // weights from measurement model
	geometry_msgs::Point vars_B_;// 3 variances B in Motion model
	double var_alpha_;

	double sample(double b);
	geometry_msgs::Point sample(geometry_msgs::Point b, geometry_msgs::Point pt);
	CatheterPose sample(CatheterPose c);
	std::vector<CatheterPose> setParticles(double i, int n);	
	double min_alpha(double a);
	double numericSolver(double i);
	geometry_msgs::Point convertEigen2Point(Eigen::Vector3d e);
	Eigen::Vector3d convertPoint2Eigen(geometry_msgs::Point p);
	void normalizeWeights();
	void sortCatheters();// sort the catheters according to the weights

};

#endif
