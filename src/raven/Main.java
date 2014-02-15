package raven;

import raven.game.RavenGame;
import raven.ui.GameCanvas;
import raven.ui.RavenUI;
import raven.utils.Log;
import raven.utils.Log.Level;
import javax.swing.SwingUtilities;

//import org.opencv.core.Core;

public class Main {
	private static RavenUI ui;
	private static RavenGame game;
	
    public static void main(String args[]) {
    	//TODO: added for slam
    	//System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    	
    	Log.setLevel(Level.DEBUG);
    	
    	game = new RavenGame();
    	SwingUtilities.invokeLater(new Runnable() {
  	      public void run() {
  	    	ui = new RavenUI(game);
  	    	GameCanvas.getInstance().setNewSize(game.getMap().getSizeX(), game.getMap().getSizeY());
  	      }
  	    });
    	//ui = new RavenUI(game);
    	//GameCanvas.getInstance().setNewSize(game.getMap().getSizeX(), game.getMap().getSizeY());
    	gameLoop();
	}
    
    public static RavenUI getUI(){return ui;};
	//////////////////////////////////////////////////////////////////////////
	// Game simulation

	private static void gameLoop() {
    	
    	Log.info("raven", "Starting game...");
    	
    	long lastTime = System.nanoTime();
    	
    	while (true) {
    		// TODO Resize UI if the map changes!
    		
    		long currentTime = System.nanoTime();

    		game.update((currentTime - lastTime) * 1.0e-9); // converts nano to seconds
    		lastTime = currentTime;
    		// Always dispose the canvas
    		//if(game.getMap() != null){
    		//if(!game.isPaused()) {
    			try {
    				//GameCanvas.startDrawing(game.getMap().getSizeX(), game.getMap().getSizeY());
    				
    				SwingUtilities.invokeLater(new Runnable() {
    			  	      public void run() {
    			  	    	GameCanvas.startDrawing();
    			  	    	game.render();
    			  	      }
    			  	    });
    				
    			} finally {
    				SwingUtilities.invokeLater(new Runnable() {
  			  	      public void run() {
  			  	    	GameCanvas.stopDrawing();
  			  	      }
  			  	    });
    				
    			}
    		//}
    		//}
    		
    		long millisToNextUpdate = (long) Math.max(0, 16.66667 - (System.nanoTime() - currentTime)*1.0e-6);
			
			try {
				Thread.sleep(millisToNextUpdate);
			} catch (InterruptedException e) {
				break;
			}
    	}
    }


}
