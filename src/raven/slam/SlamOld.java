package raven.slam;

import java.util.ArrayList;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

import raven.efkslam.Landmarks;
import raven.math.Vector2D;

public class SlamOld {

	Landmarks foundLandmarks = new Landmarks();
	private JKalmanFilterOld kf;
	private double rover_x;
	private double rover_y;
	
	public SlamOld(){
		foundLandmarks = new Landmarks();
	}
	
	public void initKalman(double rover_x, double rover_y)
	{
		this.rover_x = rover_x;
		this.rover_y = rover_y;
		
		if(!foundLandmarks.isEmpty())
		{
			int dynamic = foundLandmarks.size() * 2 + 2;
			int measured = foundLandmarks.size() * 2;
			try 
			{
				kf = new JKalmanFilterOld(dynamic, measured);
			} 
			catch (Exception e) 
			{
				e.printStackTrace();
			}
			
			Mat postState = new Mat(dynamic, 1, CvType.CV_64F);
			postState.put(0, 0, this.rover_x);
			postState.put(1, 0, this.rover_y);
			for(int i = 0; i < foundLandmarks.size(); i++){
				//double x = foundLandmarks.get(i).mapLocation.x;
				//double y = foundLandmarks.get(i).mapLocation.y;
				double x = foundLandmarks.get(i).pos.x;
				double y = foundLandmarks.get(i).pos.y;
				postState.put(i*2+2,0,x);
				postState.put(i*2+3,0,y);
			}
			kf.setState_post(postState);	
		}	
	}
	
	public void kalmanFilter(Vector2D velocity){
		Mat predicted = kf.Predict(velocity);
		
		int measured = foundLandmarks.size() * 2;
		Mat measurement = new Mat(1, measured, CvType.CV_64F);

		for(int i = 0; i < foundLandmarks.size(); i++){
			if(foundLandmarks.get(i).updatedLocation == null){
				//double x = foundLandmarks.get(i).mapLocation.x;
				//double y = foundLandmarks.get(i).mapLocation.y;
				double x = foundLandmarks.get(i).pos.x;
				double y = foundLandmarks.get(i).pos.y;
				measurement.put(0,i*2,x);
				measurement.put(0,i*2+1, y);
			}
			else{
				//double x = foundLandmarks.get(i).updatedLocation.x;
				//double y = foundLandmarks.get(i).updatedLocation.y;
				double x = foundLandmarks.get(i).pos.x;
				double y = foundLandmarks.get(i).pos.y;
				measurement.put(0,i*2,x);
				measurement.put(0,i*2+1, y);
			}
			
		}
		
		Mat corrected = kf.Correct(measurement);
		
		Mat post = kf.getState_post();
		System.out.println(post.dump());
	}

	public Landmarks getLandmarks()
	{
		return foundLandmarks;
	}
}
