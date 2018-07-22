package raven.efkslam;

import java.util.Map;
import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.ConcurrentHashMap;
import java.util.Collections;

public class PublishingRoverPose {
	private final ConcurrentMap<String, Pose> poses;
	private final Map<String, Pose> unmodifiablePoseMap;
	
	public PublishingRoverPose(Map<String, Pose> poses) {
		this.poses = new ConcurrentHashMap<String, Pose>(poses);
		this.unmodifiablePoseMap = Collections.unmodifiableMap(this.poses);
	}
	
	public Map<String, Pose> getPoses() {
		return unmodifiablePoseMap;
	}
	public Pose getPose(String id) {
		return poses.get(id);
	}
	public void setPose(String id, double x, double y) {
		if (!poses.containsKey(id)) throw new IllegalArgumentException("invalid rover name: " + id);
		poses.get(id).set(x, y);
	}
}
