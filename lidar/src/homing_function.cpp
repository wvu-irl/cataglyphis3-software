
// find correct cylinders
arma::mat xs1;
arma::mat xs2;
arma::mat ys1;
arma::mat ys2;
double dist = 2.0-12.0*0.0254;
double r = 6.0*0.0254;
double t, c1_x, c1_y, c2_x, c2_y, x, y, ax1, ay1, ax2, ay2, x_mean, y_mean, d, bearing;
double v1_x, v1_y, v2_x, v2_y, v1_mag, v2_mag, v_dot, X1s_x, X1s_y, X2s_x, X2s_y, X1s_mag, X2s_mag;
double cx1, cx2, cy1, cy2;
bool cylinder_found = false;


// rotate points and transform cylinder parameters
for (int ii=0; ii<cylinders.size(); ii++)
{
	if (cylinders[ii].axis_direction(2,0)!=0)
	{
		t = -cylinders[ii].point_in_space(2,0)/cylinders[ii].axis_direction(2,0);
		cylinders[ii].point_in_space(0,0) = cylinders[ii].point_in_space(0,0)+t*cylinders[ii].axis_direction(0,0);
		cylinders[ii].point_in_space(1,0) = cylinders[ii].point_in_space(1,0)+t*cylinders[ii].axis_direction(1,0);
		cylinders[ii].point_in_space(2,0) = 0.0;
	}
}


// find all cylinders correct distance apart
for (int ii=0; ii<cylinders.size()-1; ii++)
{

	c1_x = cylinders[ii].point_in_space(0,0);
	c1_y = cylinders[ii].point_in_space(1,0);
	for (int jj=ii+1; jj<cylinders.size(); jj++)
	{

		c2_x = cylinders[jj].point_in_space(0,0);
		c2_y = cylinders[jj].point_in_space(1,0);
		if (abs(sqrt((c1_x-c2_x)*(c1_x-c2_x)+(c1_y-c2_y)*(c1_y-c2_y))-dist)<0.05)
		{
			cylinder_found = true;
			if (c1_x*c2_y-c2_x*c1_y>0)
			{
				xs1 = cylinders[ii].points.row(0);
				ys1 = cylinders[ii].points.row(1);
				xs2 = cylinders[jj].points.row(0);
				ys2 = cylinders[jj].points.row(1);
				cx1 = c1_x;
				cy1 = c1_y;
				cx2 = c2_x;
				cy2 = c2_y;
			}
			else
			{
				xs1 = cylinders[jj].points.row(0);
				ys1 = cylinders[jj].points.row(1);
				xs2 = cylinders[ii].points.row(0);
				ys2 = cylinders[ii].points.row(1);
				cx1 = c2_x;
				cy1 = c2_y;
				cx2 = c1_x;
				cy2 = c1_y;
			}
		}
	}
}

// do the fit
int n1 = xs1.n_cols;
int n2 = xs2.n_cols;
arma::mat X(4,1);
arma::mat FX(n1+n2+1,1);
arma::mat J(n1+n2+1,4);
arma::mat W(n1+n2+1,n1+n2+1,arma::fill::eye);
arma::mat JtWJ(4,4);
W(n1+n2,n1+n2) = 1600;
bool explode = false;

if (cylinder_found)
{
	X(0,0) = cx1; //column 1 x-center
	X(1,0) = cy1; //column 1 y-center
	X(2,0) = cx2; //column 2 x-center
	X(3,0) = cy2; //column 2 y-center
	/*//ROS_INFO("Cylinder centroids = %f, %f, %f, %f", c1_x, c1_y, c2_x, c2_y);
	// alternate initial guess
	double X1s_x = arma::as_scalar(arma::mean(xs1,1));
	double X1s_y = arma::as_scalar(arma::mean(ys1,1));
	double X2s_x = arma::as_scalar(arma::mean(xs2,1));
	double X2s_y = arma::as_scalar(arma::mean(ys2,1));
	// cout<<"X1s_x="<<X1s_x<<endl;
	// cout<<"X1s_y="<<X1s_y<<endl;
	// cout<<"X2s_x="<<X2s_x<<endl;
	// cout<<"X2s_y="<<X2s_y<<endl;
	double X1s_mag = sqrt(X1s_x*X1s_x+X1s_y*X1s_y);
	double X2s_mag = sqrt(X2s_x*X2s_x+X2s_y*X2s_y);
	X1s_x = X1s_x+r*X1s_x/X1s_mag;
	X1s_y = X1s_y+r*X1s_y/X1s_mag;
	X2s_x = X2s_x+r*X2s_x/X2s_mag;
	X2s_y = X2s_y+r*X2s_y/X2s_mag;
	X(0) = X1s_x; //column 1 x-center
	X(1) = X1s_y; //column 1 y-center
	X(2) = X2s_x; //column 2 x-center
	X(3) = X2s_y; //column 2 y-center*/

	for (int ii = 0; ii<20; ii++)
	{
		ax1 = X(0,0);
		ay1 = X(1,0);
		ax2 = X(2,0);
		ay2 = X(3,0);
		for (int jj = 0; jj<n1; jj++)
		{
			x = xs1(0,jj);
			y = ys1(0,jj);
			FX(jj,0) = sqrt((x-ax1)*(x-ax1)+(y-ay2)*(y-ay1))-r; 
			J(jj,0) = (ax1-x)/sqrt((ax1-x)*(ax1-x)+(ay1-y)*(ay1-y));
			J(jj,1) = (ay1-y)/sqrt((ax1-x)*(ax1-x)+(ay1-y)*(ay1-y));
		}
		for (int jj = n1; jj<n1+n2; jj++)
		{
			x = xs2(0,jj-n1);
			y = ys2(0,jj-n1);
			FX(jj,0) = sqrt((x-ax2)*(x-ax2)+(y-ay2)*(y-ay2))-r; 
			J(jj,2) = (ax2-x)/sqrt((ax2-x)*(ax2-x)+(ay2-y)*(ay2-y));
			J(jj,3) = (ay2-y)/sqrt((ax2-x)*(ax2-x)+(ay2-y)*(ay2-y));
		}
		FX(n1+n2,0) = sqrt((ax1-ax2)*(ax1-ax2)+(ay1-ay2)*(ay1-ay2))-dist;
		J(n1+n2,0) = (ax1-ax2)/sqrt((ax1-ax2)*(ax1-ax2)+(ay1-ay2)*(ay1-ay2));
		J(n1+n2,1) = (ay1-ay2)/sqrt((ax1-ax2)*(ax1-ax2)+(ay1-ay2)*(ay1-ay2));
		J(n1+n2,2) = (ax2-ax1)/sqrt((ax1-ax2)*(ax1-ax2)+(ay1-ay2)*(ay1-ay2));
		J(n1+n2,3) = (ay2-ay1)/sqrt((ax1-ax2)*(ax1-ax2)+(ay1-ay2)*(ay1-ay2));
		JtWJ = J.st()*W*J;
		if (!JtWJ.has_nan() && !JtWJ.has_inf())
		{
			X = X-0.25*arma::solve(JtWJ,J.st()*W*FX);
		}
		else
		{
			explode = true;
		}
	}

	//cout << "5" << endl;
	// this part is used for comparing the results between two parts
	/*x_mean = (X(0,0)+X(2,0))/2;
	y_mean = (X(1,0)+X(3,0))/2;

	d = sqrt(x_mean*x_mean+y_mean*y_mean);

	v2_x = X(0,0)-X(2,0);
	v2_y = X(1,0)-X(3,0);

	v1_mag = sqrt(x_mean*x_mean+y_mean*y_mean); 
	v2_mag = sqrt(v2_x*v2_x+v2_y*v2_y); 
	v1_x = x_mean/v1_mag;
	v1_y = y_mean/v1_mag;
	v2_x = v2_x/v2_mag;
	v2_y = v2_y/v2_mag;

	v_dot = v1_x*v2_x+v1_y*v2_y;
	bearing = acos(v_dot)-3.14159265358979323846264338327950288419716939937510582097494459230781640628620899862803482534211706798214808651/2;
	double x_est_k = d*cos(bearing);
	double y_est_k = d*sin(bearing);
	double b_h_diff_k = atan2(-v1_y,-v1_x); 
	double heading_est_k = -(b_h_diff-bearing);*/



	x_mean = (cx1+cx2)/2;
	y_mean = (cy1+cy2)/2;
	d = sqrt(x_mean*x_mean+y_mean*y_mean);

	v2_x = cx1-cx2;
	v2_y = cy1-cy2;
	v1_mag = sqrt(x_mean*x_mean+y_mean*y_mean); 
	v2_mag = sqrt(v2_x*v2_x+v2_y*v2_y); 
	v1_x = x_mean/v1_mag;
	v1_y = y_mean/v1_mag;
	v2_x = v2_x/v2_mag;
	v2_y = v2_y/v2_mag;

	v_dot = v1_x*v2_x+v1_y*v2_y;
	bearing = acos(v_dot)-3.14159265358979323846264338327950288419716939937510582097494459230781640628620899862803482534211706798214808651/2;
	double x_est = d*cos(bearing);
	double y_est = d*sin(bearing);
	double b_h_diff = atan2(-v1_y,-v1_x); 
	double heading_est = -(b_h_diff-bearing);

	ROS_INFO("\nHOMING UPDATE!");
	ROS_INFO("x = %f", x_est);
	ROS_INFO("y = %f", y_est);
	ROS_INFO("heading = %f", heading_est*180.0/3.14159265);

	_homing_x=x_est;
	_homing_y=y_est;
	_homing_heading=heading_est;
	_homing_found=true;
	ROS_INFO("********************");
	ROS_INFO("x_est = %f",x_est);
	ROS_INFO("y_est = %f",y_est);
	ROS_INFO("x_mean = %f",x_mean);
	ROS_INFO("y_mean = %f",y_mean);
	ROS_INFO("heading = %f\n",heading_est);
	ROS_INFO("explode = %f\n",explode);
}
else
{
	_homing_x=0;
	_homing_y=0;
	_homing_heading=0;
	_homing_found=false;
} //end if cylinder found
double diff1, diff2;
diff1 = sqrt((cx1-X(0,0))*(cx1-X(0,0))+(cy1-X(1,0))*(cy1-X(1,0)));
diff2 = sqrt((cx2-X(2,0))*(cx2-X(2,0))+(cy2-X(3,0))*(cy2-X(3,0)));

if(stopSavingDataToFile==false && _homing_found==true && (diff1+diff2<0.3 || explode == true))
{
	for (int i=0; i<points_cluster.size(); i++)
	{
		for (int jj=0; jj<cloud_cylinder->points.size(); jj++)
		{
			outputFile << cylinders[i].points(0,jj) << ",";
			outputFile << cylinders[i].points(1,jj) << ",";
			outputFile << cylinders[i].points(2,jj) << ",";
			outputFile << i << ","; //cylinder number
			outputFile << cylinders[i].point_in_space(0,0) << ",";
			outputFile << cylinders[i].point_in_space(1,0) << ",";
			outputFile << cylinders[i].point_in_space(2,0) << ",";
			outputFile << cylinders[i].axis_direction(0,0) << ",";
			outputFile << cylinders[i].axis_direction(1,0) << ",";
			outputFile << cylinders[i].axis_direction(2,0) << ",";
			outputFile << cylinders[i].raius_estimate(0,0);
			outputFile << std::endl; 	
		}
	}
	stopSavingDataToFile=true; 
}
else
{
	outputFile.close();
}
