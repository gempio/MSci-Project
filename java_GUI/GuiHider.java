import java.util.*;
import java.util.concurrent.LinkedBlockingQueue;
import java.io.*;

public class GuiHider implements Listener{

	HashMap<Integer,String[]> treasures;
	CommandObject command;
	Connector r;
	public static void main(String[] args) {
		new GuiHider();
	}

	public GuiHider() {
		command = new CommandObject();
		register(this.command);
		treasures = new HashMap<Integer,String[]>();
		treasures.put(0, setTreasure("Red","Ball"));
		treasures.put(1, setTreasure("Blue","Hat"));
		treasures.put(2, setTreasure("Red", "Square"));
		treasures.put(3, setTreasure("Green", "Hat"));
		treasures.put(4, setTreasure("Yellow","Bottle"));
		treasures.put(5, setTreasure("Orange", "Bottle"));
		treasures.put(6, setTreasure("Pink", "Hat"));
		treasures.put(7, setTreasure("Violet", "Square"));


		r = new Connector("127.0.1.1", 6009, this.command);
		Thread r2 = new Thread(r);
		r2.start();
		
		waitForServer(r);//Wait for the server to start up before continuing.

		r.setId("Hider");
	}

	//An empty blocking method that ensures that server has time to set up before continuing running asynchrously.
	public void waitForServer(Connector r) {
		while(!(r.isRunning())){
			System.out.print("");
		} //Wait for the server to start up before continuing.
	}


	//Simple method that sends a send robot message and asks the server to pass it through.
	public void sendTresureFirstProperty(Connector r, int room) {
		r.sendMessage("%%found Hider TabUI " + room + " " + room + " \"(colour " + treasures.get(room)[0] + ")\"");
	}
	//A method that returns an array of two property treasure.
	public String[] setTreasure(String colour, String shape) {
		String[] temp = {colour,shape};
		return temp;
	}

	public void register(Observable observable) {observable.add(this);}
  	public void unregister(Observable observable) {observable.remove(this);}

  	public void fieldChanged(Object source, String attribute) {
  		if(attribute.contains("error")) {
  			String[] temp = attribute.split("\"");
  			int room = Integer.parseInt(temp[1]);
  			sendTresureFirstProperty(r, room);
  		}
    	System.out.println("Hider GUI: " + attribute); // this has to be implemented
  	}
}