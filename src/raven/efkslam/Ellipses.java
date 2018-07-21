package raven.efkslam;

import java.util.ArrayList;
import java.util.List;
import java.util.Iterator;
//import raven.math.Vector2D;
import raven.ui.GameCanvas;

public class Ellipses {
	private List<Ellipse> ellipses = new ArrayList<Ellipse>(100);
	
	public void addEllipse(Ellipse ellipse){
		ellipses.add(ellipse);
	}
	public void addEllipse(double centerX, double centerY, double majorAxis, double minorAxis, double angleDeg){
		Ellipse ellipse = new Ellipse(centerX, centerY, majorAxis, minorAxis, angleDeg);
		ellipses.add(ellipse);
	}
	public Iterator<Ellipse> iterator() {
		return ellipses.iterator();
	}
	public void clear(){
		ellipses.clear();
	}
	public int size(){
		return ellipses.size();
	}
	public void render(){
		for(Ellipse ellipse : ellipses){
			ellipse.render();
		}
	}
}

class Ellipse{
	double centerX;
	double centerY;
	double majorAxis;
	double minorAxis;
	double angleDeg; // angle in degrees (0 to 360)
	public Ellipse(double centerX, double centerY, double majorAxis, double minorAxis, double angleDeg){
		this.centerX = centerX; this.centerY = centerY; this.majorAxis = majorAxis; this.minorAxis = minorAxis; this.angleDeg = angleDeg;
	}
	public void render(){
		GameCanvas.ellipseDeg(centerX, centerY, majorAxis, minorAxis, angleDeg);
	}
}