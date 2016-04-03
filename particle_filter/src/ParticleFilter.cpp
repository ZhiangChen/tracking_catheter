#include <particle_filter/ParticleFilter.h>
#include <math.h>

ParticleFilter::ParticleFilter(int n):nm_(n)
{
	i_=0;
	i_state_ = true;
	Catheters_.resize(nm_);
	weights_.resize(nm_);
	srand(time(NULL));
}

ParticleFilter::ParticleFilter(double min_c, double max_c, double step, int n)
{
	double number;
	number = ((max_c-min_c)/step+1)*n;
	if (number > 2000000000)
		overflow_ = true;
	else 
		overflow_ = false;

	nm_ = int(((max_c-min_c)/step+1)*n);
	min_i_ = min_c;
	step_i_ = step;
	i_ = min_i_;
	i_state_=false;
	n_ = n;
	Catheters_.resize(nm_);
	weights_.resize(nm_);
	srand(time(NULL));	
	ROS_WARN("constructor finished, the number of catheters is %d", nm_);
}


bool ParticleFilter::setWeights(std::vector<double> w)
{
	if (w.size()!=Catheters_.size())
	{
		return false;
	}
	weights_=w;
	return true;
}

void ParticleFilter::setMotionErrors(geometry_msgs::Point vars_B, double var_alpha)
{
	vars_B_ = vars_B;
	var_alpha_ = var_alpha;
}

void ParticleFilter::displayCatheter(double index)
{
	std::cout<<"The No."<<index+1<<" catheter pose:"<<std::endl;
	std::cout<<"alpha :"<<Catheters_[index].alpha_<<std::endl;
	std::cout<<"A("<<Catheters_[index].A_.x<<", "<<Catheters_[index].A_.y<<", "<<Catheters_[index].A_.z<<")"<<std::endl;
	std::cout<<"B("<<Catheters_[index].B_.x<<", "<<Catheters_[index].B_.y<<", "<<Catheters_[index].B_.z<<")"<<std::endl;
	std::cout<<"C("<<Catheters_[index].C_.x<<", "<<Catheters_[index].C_.y<<", "<<Catheters_[index].C_.z<<")"<<std::endl;
	std::cout<<"t("<<Catheters_[index].t_.x<<", "<<Catheters_[index].t_.y<<", "<<Catheters_[index].t_.z<<")"<<std::endl<<std::endl;
}

bool ParticleFilter::generateParticles(double i)
{
	if (i_state_ == false)
	{
		ROS_ERROR("wrong generateParticles fnc!");
		return false;
	}
	i_=i; // update the current in coil
	Catheters_ = setParticles(i_,nm_);
	return true;
}

bool ParticleFilter::generateParticles()
{
	if (i_state_ == true)
	{
		ROS_ERROR("wrong generateParticles fnc!");
		return false;
	}
	if (overflow_ == true)
	{
		ROS_ERROR("the number of particles is overflow!");
		return false;
	}
	i_=min_i_;
	std::vector<CatheterPose> cp;
	cp.resize(n_);
	for (int j=0; j<nm_; j=j+n_)
	{
		std::cout<<"The current is "<<i_<<std::endl;
		cp = setParticles(i_,n_);
		for(int k=0; k<n_; k++)
		{
			Catheters_[j+k]=cp[k];
		//	std::cout<<j+k<<std::endl;
		}

		i_ = i_ + step_i_;
	}
	return true;
}

void ParticleFilter::icr_generateParticles(double d_i)
{
	double a;
	double df_da;
	double df_di;
	double alpha_z;
	Eigen::Vector3d A_e,B_e,C_e,t_e;
	Eigen::Vector3d g(9.8,0,0);
	double B_y = B(1);
	double B_x = B(0); 
	double gr = g(0);

	A_e<<0,0,0;

	for (int j=0 ; j<nm_; j++)
	{
		a = Catheters_[j].alpha_; 
		df_di = N*A*(B_y*cos(a)-B_x*sin(a));
		df_da = N*A*(-B_y*sin(a)-B_x*cos(a))
				-l_AB*m*gr*(a*sin(a)+cos(a)-1)/a/a - E*I;
		alpha_z = a -df_di/df_da*d_i;
		t_e<<cos(alpha_z),sin(alpha_z),0;
		B_e<<l_AB/alpha_z*sin(alpha_z),l_AB/alpha_z*(1-cos(alpha_z)),0;
		C_e<<B_e + l_BC*t_e;

		Catheters_[j].A_ = convertEigen2Point(A_e);
		Catheters_[j].B_ = convertEigen2Point(B_e);
		Catheters_[j].t_ = convertEigen2Point(t_e);
		Catheters_[j].C_ = convertEigen2Point(C_e);
		Catheters_[j].alpha_ = alpha_z;

		Catheters_[j] = sample(Catheters_[j]);
	}
}

std::vector<CatheterPose> ParticleFilter::setParticles(double i, int n)
{

	std::vector<CatheterPose> cp;
	cp.resize(n);
	
	double alpha_z = numericSolver(i);
	CatheterPose Catheter;
	Eigen::Vector3d A,B,C,t;
	t<<cos(alpha_z),sin(alpha_z),0;
	A<<0,0,0;
	B<<l_AB/alpha_z*sin(alpha_z),l_AB/alpha_z*(1-cos(alpha_z)),0;
	C<<B + l_BC*t;

	Catheter.A_=convertEigen2Point(A);
	Catheter.B_=convertEigen2Point(B);
	Catheter.t_=convertEigen2Point(t);
	Catheter.C_ = convertEigen2Point(C);
	Catheter.alpha_ = alpha_z;

	
	//Catheter.displayCatheter();

	for (int j=0; j < n; j++)
	{
		cp[j]=sample(Catheter);
	}

	return cp;
}

geometry_msgs::Point ParticleFilter::convertEigen2Point(Eigen::Vector3d e)
{
	geometry_msgs::Point p;
	p.x=e(0);
	p.y=e(1);
	p.z=e(2);
	return p;
}

Eigen::Vector3d ParticleFilter::convertPoint2Eigen(geometry_msgs::Point p)
{
	Eigen::Vector3d e;
	e(0)=p.x;
	e(1)=p.y;
	e(2)=p.z;
	return e;
}

double ParticleFilter::numericSolver(double i)
{
	double alpha_z; // initial solution is -3.14
	double alpha_z_n;
	Eigen::Vector3d t_B;
	Eigen::Vector3d M_z;
	Eigen::Vector3d j(0,1,0);
	Eigen::Vector3d g(9.8,0,0);
	double err=100;
	
	if (i>0.001)
	{	
		alpha_z=1.56;
		alpha_z_n=1.56;
		while(err>0.005)
		{
			t_B<<cos(alpha_z),sin(alpha_z),0;
		//	ROS_INFO_STREAM("t_B:"<<t_B.transpose()<<std::endl);
			M_z= N*i*A*t_B.cross(B) + l_AB/alpha_z*(1-cos(alpha_z))*m*j.cross(g);

		//	Eigen::Vector3d M_B; 
		//	M_B= N*i*A*t_B.cross(B);
		//	ROS_INFO_STREAM("M_B:"<<M_B.transpose()<<std::endl);

		//	Eigen::Vector3d M_G;
		//	M_G= l_AB/alpha_z*(1-cos(alpha_z))*m*j.cross(g);
		//	ROS_INFO_STREAM("M_G:"<<M_G.transpose()<<std::endl);
			
			alpha_z_n=M_z(2)/E/I;
			err=alpha_z-alpha_z_n;
			if (err<0)
				err=-err; // abs() is bad
		//	 ROS_INFO("err is %f",err);
		//	 ROS_INFO("alpha_z: %f",alpha_z);
		//	 ROS_INFO("alpha_z_n: %f",alpha_z_n);
		//	 ROS_INFO("M_z : %f", M_z(2));
			alpha_z-=0.0002;
		/*	static double dd=0;
			dd++;
			if (dd>1)
				std::cin>>dd;*/
			if (alpha_z < 0)
			{
				ROS_ERROR("No sulution!");
				return 0;
			}
		}
	}
	else if (i>-0.001)
	{
		return 0.0000000001;
	}
	else
	{
		alpha_z=-1.56;
		alpha_z_n=-1.56;
		while(err>0.005)
		{
			t_B<<cos(alpha_z),sin(alpha_z),0;
		//	ROS_INFO_STREAM("t_B:"<<t_B.transpose()<<std::endl);

			M_z= N*i*A*t_B.cross(B) + l_AB/alpha_z*(1-cos(alpha_z))*m*j.cross(g);

		//	Eigen::Vector3d M_B; 
		//	M_B= N*i*A*t_B.cross(B);
		//	ROS_INFO_STREAM("M_B:"<<M_B.transpose()<<std::endl);

		//	Eigen::Vector3d M_G;
		//	M_G= l_AB/alpha_z*(1-cos(alpha_z))*m*j.cross(g);
		//	ROS_INFO_STREAM("M_G:"<<M_G.transpose()<<std::endl);

			alpha_z_n=M_z(2)/E/I;
			err=alpha_z_n-alpha_z;
			if (err<0)
				err=-err; 
		//	ROS_INFO("err is %f",err);
		//	ROS_INFO("alpha_z: %f",alpha_z);
		//	ROS_INFO("alpha_z_n: %f",alpha_z_n);
			alpha_z+=0.0002;
			if (alpha_z > 0)
			{
				ROS_ERROR("No sulution!");
				return 0;
			}
		}
	}
	//std::cout<<"alpha_z: "<<alpha_z<<std::endl;
	return alpha_z;
}

double ParticleFilter::min_alpha(double a) {
	ROS_INFO("minminmin");
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

double ParticleFilter::sample(double b)
{
	double x=0, r=0;
	for(int j=0; j < 12; j++)
	{
		r=(rand()*2.0/RAND_MAX-1.0)*b;
		x=x+r;
	}
	return x/2;
}

geometry_msgs::Point ParticleFilter::sample(geometry_msgs::Point b, geometry_msgs::Point pt)
{
	geometry_msgs::Point pt_h;
	pt_h.x = pt.x+sample(b.x);
	pt_h.y = pt.y+sample(b.y);
	pt_h.z = pt.z+sample(b.z);
	return pt_h;
}

CatheterPose ParticleFilter::sample(CatheterPose c)
{
	CatheterPose c_n;
	c_n.A_=c.A_;
	c_n.B_=sample(vars_B_,c.B_);
	c_n.alpha_ = c.alpha_ + sample(var_alpha_);

	c_n.t_.x = cos(c_n.alpha_);
	c_n.t_.y = sin(c_n.alpha_);
	c_n.t_.z = 0;

	c_n.C_.x=c_n.B_.x + l_BC*c_n.t_.x;
	c_n.C_.y=c_n.B_.y + l_BC*c_n.t_.y;
	c_n.C_.z=c_n.B_.z + l_BC*c_n.t_.z;

	return c_n;
}

void ParticleFilter::normalizeWeights()
{
	double n=0;
	for(int j=0;j<nm_;j++)
	{
		n+=weights_[j];
	}
	for(int j=0;j<nm_;j++)
	{
		weights_[j]=weights_[j]/n;
		//std::cout<<"weight No. "<<j<<" : "<<weights_[j]<<std::endl;
	}
}

void ParticleFilter::sortCatheters()
{
    // bubble_sort 
    int k=0;
	for (int j=0; j< nm_; j++)
	{
		for (int l=nm_; l>k; l--)
		{
			if(weights_[l]>weights_[l-1])
			{
				double w_temp = weights_[l-1];
				weights_[l-1]=weights_[l];
				weights_[l]=w_temp;
				CatheterPose c_temp = Catheters_[l-1];
				Catheters_[l-1] = Catheters_[l];
				Catheters_[l] = c_temp;
			}
		}
		k++;
	}

/*	for(int j=0;j<nm_;j++)
	{
		std::cout<<"The catheter No."<<j+1<<": "<<weights_[j]<<std::endl;
		Catheters_[j].displayCatheter();
	}
*/	
}

void ParticleFilter::resample()
{
	normalizeWeights();
	sortCatheters();
	// reassign the samples
	std::vector<int> renm;
	renm.resize(nm_);
	int t=0;
	for (int j=0;j<nm_;j++)
	{
		renm[j]=int(weights_[j]*nm_);
		t+=renm[j];
	}
	//std::cout<<"t: "<<t<<std::endl;
	int r=nm_-t;
	for (int j=0; j<r; j++)
	{
		renm[j]++;
	}

	int index=nm_-1;
	for (int j=0;j<nm_;j++)
	{
		while(renm[j]>1)
		{
			renm[j]--;
			Catheters_[index]=Catheters_[j];
			renm[index]++;
			weights_[index]=weights_[j]; 
			index--;
		}
	}
	sortCatheters();
}

int ParticleFilter::resample2()
{
	normalizeWeights();
	sortCatheters();
	std::vector<int> renm;
	renm.resize(nm_);
	int t=0,id=0;
	for (int j=0;j<nm_;j++)
	{
		renm[j]=int(weights_[j]*nm_);
		t+=renm[j];
		if(renm[j]>0)
			id++;
	}
	nm_=t;// update the number of particles
	std::vector<CatheterPose> Catheters_temp = Catheters_;
	std::vector<double> weights_temp = weights_;
	Catheters_.clear();
	weights_.clear();
	Catheters_.resize(nm_);
	weights_.resize(nm_);
	for (int j=0; j<id; j++)
	{
		Catheters_[j]=Catheters_temp[j];
		weights_[j]=weights_temp[j];
		//std::cout<<weights_[j]<<std::endl;
		//Catheters_[j].displayCatheter();
	}

	int index=nm_-1;
	for (int j=0;j<nm_;j++)
	{
		while(renm[j]>1)
		{
			renm[j]--;
			Catheters_[index]=Catheters_[j];
			renm[index]++;
			weights_[index]=weights_[j]; 
			index--;
		}
	}

	sortCatheters();

	return nm_;

}

CatheterPose ParticleFilter::getBestParticle(double &weight)
{
	weight=weights_[0];
	return Catheters_[0];
}

CatheterPose ParticleFilter::getBestParticle()
{
	return Catheters_[0];
}

CatheterPose ParticleFilter::getMeanParticle()
{
	CatheterPose c;
	for (int j=0;j<nm_;j++)
	{
		c=c+Catheters_[j];
	}
	return c/nm_;
}

std::vector<double> ParticleFilter::getVar()
{
	CatheterPose cm;
	cm = getMeanParticle();
	std::vector<double> v,v_t;
	v.resize(2);
	v_t.resize(2);
	for (int j=0; j<nm_; j++)
	{
		v = Catheters_[j]-cm;
		for (int k=0; k<2; k++)
		{
			v[k]=v[k]*v[k];
			v_t[k]=v_t[k]+v[k];
		}
	}
	for (int j=0; j<2; j++)
	{
		v_t[j]=sqrt(v_t[j]/nm_);
	}

	return v_t;
}

CatheterPose CatheterPose::operator + (CatheterPose &c2)
{
	CatheterPose c;
	//c.A_ = (A_+c2.A_);
	c.A_.x = (A_.x+c2.A_.x);
	c.A_.y = (A_.y+c2.A_.y);
	c.A_.z = (A_.z+c2.A_.z);
	//c.B_ = (B_+c2.B_);
	c.B_.x = (B_.x+c2.B_.x);
	c.B_.y = (B_.y+c2.B_.y);
	c.B_.z = (B_.z+c2.B_.z);
	//c.C_ = (C_+c2.C_);
	c.C_.x = (C_.x+c2.C_.x);
	c.C_.y = (C_.y+c2.C_.y);
	c.C_.z = (C_.z+c2.C_.z);
	//c.t_ = (D_+c2.t_);
	c.t_.x = (t_.x+c2.t_.x);
	c.t_.y = (t_.y+c2.t_.y);
	c.t_.z = (t_.z+c2.t_.z);
	//alpha
	c.alpha_ = alpha_+c2.alpha_;
	return c;
}

CatheterPose CatheterPose::operator / (double n)
{
	CatheterPose c;
	c.A_.x = (A_.x)/n;
	c.A_.y = (A_.y)/n;
	c.A_.z = (A_.z)/n;

	c.B_.x = (B_.x)/n;
	c.B_.y = (B_.y)/n;
	c.B_.z = (B_.z)/n;

	c.C_.x = (C_.x)/n;
	c.C_.y = (C_.y)/n;
	c.C_.z = (C_.z)/n;

	c.t_.x = (t_.x)/n;
	c.t_.y = (t_.y)/n;
	c.t_.z = (t_.z)/n;

	c.alpha_ = alpha_/n;
	return c;
}

std::vector<double> CatheterPose::operator - (CatheterPose &c2)
{
	std::vector<double> v;
	double x;
	double y;
	double z;
	double d;

	x = B_.x - c2.B_.x;
	y = B_.y - c2.B_.y;
	z = B_.z - c2.B_.z;
	d = sqrt(x*x+y*y+z*z);
	v.push_back(d);

	d = alpha_ - c2.alpha_;
	v.push_back(d);

	return v;
}

geometry_msgs::Point CatheterPose::getBSpline(double s)
{
	geometry_msgs::Point b;
	b.x=-1000;
	b.y=-1000;
	b.z=-1000;
	
	if (s>1) 
		return b;
	else if (s>0.5)
	{
		s=s*2-1;
		double P1_x=B_.x;
		double P1_y=B_.y;
		double P1_z=B_.z;
		double P3_x=C_.x;
		double P3_y=C_.y;
		double P3_z=C_.z;
		double alpha2= beta2*sqrt((P3_x-P1_x)*(P3_x-P1_x)+(P3_y-P1_y)*(P3_y-P1_y)+(P3_z-P1_z)*(P3_z-P1_z));
		double P2_x=P1_x+alpha2*t_.x;
		double P2_y=P1_y+alpha2*t_.y;
		double P2_z=P1_z+alpha2*t_.z;
		b.x=P1_x*(1-s)*(1-s)+P2_x*2*(1-s)*s+P3_x*s*s;
		b.y=P1_y*(1-s)*(1-s)+P2_y*2*(1-s)*s+P3_y*s*s;
		b.z=P1_z*(1-s)*(1-s)+P2_z*2*(1-s)*s+P3_z*s*s;
		return b;
	}
	else if (s>0 || s == 0)
	{
		s=s*2;
		geometry_msgs::Point t_A;
		t_A.x=1;
		t_A.y=0;
		t_A.z=0;
		double P1_x=A_.x;
		double P1_y=A_.y;
		double P1_z=A_.z;
		double P4_x=B_.x;
		double P4_y=B_.y;
		double P4_z=B_.z;
		double alpha1= beta1*sqrt((P4_x-P1_x)*(P4_x-P1_x)+(P4_y-P1_y)*(P4_y-P1_y)+(P4_z-P1_z)*(P4_z-P1_z));
		double P2_x=P1_x+alpha1*t_A.x;
		double P2_y=P1_y+alpha1*t_A.y;
		double P2_z=P1_z+alpha1*t_A.z;
		double P3_x=P4_x-alpha1*t_.x;
		double P3_y=P4_y-alpha1*t_.y;
		double P3_z=P4_z-alpha1*t_.z;
		b.x=P1_x*(1-s)*(1-s)*(1-s)+P2_x*3*(1-s)*(1-s)*s+P3_x*3*(1-s)*s*s+P4_x*s*s*s;
		b.y=P1_y*(1-s)*(1-s)*(1-s)+P2_y*3*(1-s)*(1-s)*s+P3_y*3*(1-s)*s*s+P4_y*s*s*s;
		b.z=P1_z*(1-s)*(1-s)*(1-s)+P2_z*3*(1-s)*(1-s)*s+P3_z*3*(1-s)*s*s+P4_z*s*s*s;
		return b;
	}
	else 
		return b;
} 

void CatheterPose::displayCatheter()
{
	std::cout<<"The catheter pose:"<<std::endl;
	std::cout<<"alpha :"<<alpha_<<std::endl;
	std::cout<<"A("<<A_.x<<", "<<A_.y<<", "<<A_.z<<")"<<std::endl;
	std::cout<<"B("<<B_.x<<", "<<B_.y<<", "<<B_.z<<")"<<std::endl;
	std::cout<<"C("<<C_.x<<", "<<C_.y<<", "<<C_.z<<")"<<std::endl;
	std::cout<<"t("<<t_.x<<", "<<t_.y<<", "<<t_.z<<")"<<std::endl<<std::endl;
}
