package raven.efkslam;

public class Pose {
	double x = 0; // x coord
	double y = 0; // y coord
	//double Q = 0; // steer angle, rads
	//public Pose(double x, double y, double Q){this.x = x; this.y = y; this.Q = Q;}	
	public Pose(){};
	
	public void set(double x, double y) {
		this.x = x;
		this.y = y;
	}
	//void setX(double x) {this.x = x;}
	//void setY(double y) {this.y = y;}
	//void setQ(double Q) {this.Q = Q;}
}
