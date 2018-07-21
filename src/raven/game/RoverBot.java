package raven.game;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import raven.efkslam.ConfigFile;
import raven.efkslam.EfkSlamSim;
import raven.efkslam.Landmarks;
import raven.efkslam.Landmarks.Landmark;
import raven.game.Waypoints;
import raven.game.Waypoints.Wpt;
import raven.game.interfaces.IRavenBot;
import raven.game.navigation.NavGraphEdge;
import raven.game.navigation.PathEdge;
import raven.goals.Goal;
import raven.goals.GoalComposite;
import raven.goals.Goal_PIDFollowPath;
import raven.goals.GoalRoverThink;
import raven.math.Vector2D;
import raven.math.RandUtils;
import raven.slam.Slam;
import raven.ui.GameCanvas;

import raven.utils.PIDcontroller;
import raven.utils.Regulator;

public class RoverBot extends RavenBot {
	
	//protected GoalRoverThink reason;
	private final double brakingRate = 20; // pixel/sec^2
	private final double accelRate = 30; // pixel/sec^2
	
	private double steeringDrift = 0.05;
	private double steeringNoise = 0.01;
	private double distanceNoise = 0.001;
	private double frictCoeff = 0.5;
	private double weight = mass*9.81;
	private double frictForceMag = weight*frictCoeff;
	private double speed = 0;
	private boolean doPID = false;
	protected PIDcontroller pid = new PIDcontroller(0.7f, 0.8f, 0.1f);
	protected Waypoints track = new Waypoints();
	
	protected Regulator trackRegulator = new Regulator(200);
	
	public RoverBot(RavenGame world, Vector2D pos, Goal.GoalType mode) {
		super(world, pos, mode);
		steering.wallAvoidanceOff();
		steering.separationOff();
		track.setColor(Color.RED);
		track.setDrawPoint(false);
	}
	public void setNoise(double steerDrift, double steerNoise, double distNoise) {
		steeringDrift = steerDrift;
		steeringNoise = steerNoise;
		distanceNoise = distNoise;
	}
	public void addWptsGoal(Waypoints wpts){
		if(wpts.size() > 1) {
			List<PathEdge>  m_Path = new ArrayList<PathEdge>();
			Vector2D src = wpts.get(0).pos;
			Vector2D dest = null;
			for(int i=1; i < wpts.size(); i++) {
				Waypoints.Wpt wpt = wpts.get(i);
				dest = wpt.pos;
				PathEdge edge = new PathEdge(src, dest, NavGraphEdge.NORMAL, 0);
				m_Path.add(edge);
				System.out.println("Edge " + i + " src: " + src.toString() + " dest: " + dest.toString());
				src = dest;
			}
			Goal_PIDFollowPath g = new Goal_PIDFollowPath(this, m_Path);
			brain.AddSubgoal(g);
		}
	}
	
	public void startPid(){doPID = true;}
	public void stopPid(){doPID = false;}
	
	/*
	 * Calculation of Cross Track Error as a difference between desired course and current bearing (in degrees)
	 * 
	 */
	private float getCTE(){
		Vector2D tgt = steering.target();
		if(tgt == null) return 0;
		else {
			float bearing = (float)Math.atan2(tgt.y - position.y, tgt.x - position.x);
			float error = bearing - steering.course;
			error = (float)Math.toDegrees((float)error);
			System.out.println("Course " + Math.toDegrees(steering.course) + ", bearing " + Math.toDegrees(bearing) + ", error " + error);
			return error;
		}
	}
	
	/**
	 * this method is called from the update method. It calculates and applies
	 * the steering force for this time-step.
	 * delta is in seconds
	 */
	@Override
	protected void updateMovement(double delta) { // delta in seconds
		// (2do) pid control, acceleration, deceleration depending on doPID value
		
		if(!doPID && speed == 0) return;
		// apply steering noise and drift
		double steerAngle = Math.atan2(velocity.y, velocity.x);
		double steerAngleDeg = Math.toDegrees(steerAngle);
		double noiseSteerAngleDeg = RandUtils.nextGaussian(steerAngleDeg, steeringNoise);
		noiseSteerAngleDeg += steeringDrift;
		
		if (doPID) {
			if(speed < maxSpeed*1.0) {
				speed += accelRate*delta;
				System.out.println("Accelerating to " + speed);
				}
			
			trackRegulator.update(delta);
			if(trackRegulator.isReady()) 
			{
				Vector2D pt = new Vector2D();
				pt.x = position.x;
				pt.y = position.y;
				track.addWpt(pt);
				
				System.out.println("Added track point at= x:" + position.x + ", y:" + position.y);
			}
			float error = getCTE();
			float out = pid.pidCycle(error, (float)delta);
			float turnRate = out*2.0f; //  deg/sec
			noiseSteerAngleDeg += turnRate*delta;
		}
		else {
			System.out.println("No PID");
			if( speed > 1){speed -= brakingRate*delta;}
			else {
				speed = 0;
				System.out.println("Number of track points = " + track.size());
				return;
			}
		}
		double noiseSteerAngle = Math.toRadians(noiseSteerAngleDeg);
		heading.x = Math.cos(noiseSteerAngle);
		heading.y = Math.sin(noiseSteerAngle);
		side = heading.perp();
		
		double velX = heading.x*speed;
		double velY = heading.y*speed;
		
		// calculate delta distance due to distance noise
		double distNoise = RandUtils.nextGaussian(0, distanceNoise);
		
		velocity.x = velX;
		velocity.y = velY;
		position.x += velX*delta + distNoise*heading.x;
		position.y += velY*delta + distNoise*heading.y;
	}
	
	public Waypoints getTrack()
	{
		return this.track;
	}
}
