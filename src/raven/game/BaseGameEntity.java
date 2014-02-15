package raven.game;

import raven.game.messaging.Telegram;
import raven.math.Vector2D;

public abstract class BaseGameEntity {
	
	public static final int DEFAULT_ENTITY_TYPE = -1;
	
	private int ID;
	
	private RavenObject type;
	
	transient private boolean tag;
	
	transient private static int nextValidID;
	
	private void setID(int val) {
		if (val < nextValidID)
			throw new RuntimeException("Invalid ID during creation!");
		
		ID = val;
		
		nextValidID++;
	}
	
	protected Vector2D position;
	protected Vector2D scale;
	
	protected double boundingRadius;
	
	protected BaseGameEntity(int id) {
		setID(id);
		EntityManager.registerEntity(this);
	}

	public void update(double delta) {}
	public abstract void render();
	
	public boolean handleMessage(final Telegram msg) { return false; }
	
	public static int getNextValidID() { return nextValidID; }
	
	public static void resetNextValidID() { nextValidID = 0; }
	
	// Accessors
	
	public Vector2D pos() { return position; }
	public void setPos(Vector2D pos) { position = pos; }
	
	public double getBRadius() { return boundingRadius; }
	public void setBRadius(double r) { boundingRadius = r; }
	
	public int ID() { return ID; }
	
	public boolean isTagged() { return tag; }
	public void tag() { tag = true; }
	public void unTag() { tag = false; }
	
	public Vector2D scale() { return scale; }
	public void setScale(Vector2D val) { boundingRadius *= Math.max(val.x, val.y) / Math.max(scale.x, scale.y); scale = val; }
	public void setScale(double val) { boundingRadius *= (val / Math.max(scale.x, scale.y)); scale = new Vector2D(val, val); }
	
	public RavenObject entityType() { return type; }
	public void setEntityType(RavenObject newType) { type = newType; }
}
