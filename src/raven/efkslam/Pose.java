package raven.efkslam;

public class Pose {
	final double x; // x coord
	final double y; // y coord
	final double Q; // steer angle, rads
	Pose(double x, double y, double Q){this.x = x; this.y = y; this.Q = Q;}	
}
