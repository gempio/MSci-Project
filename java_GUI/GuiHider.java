import java.util.*;
import java.util.concurrent.LinkedBlockingQueue;
import java.io.*;

public class GuiHider implements Listener{

	HashMap<Integer,String[]> treasures;
	CommandObject command;
	Connector r;
	int rooms;
	ArrayList<String[]> treasuresMap;

	public static void main(String[] args) {
		new GuiHider();
	}

	public GuiHider() {
		//Set up for the Hider environment.
		treasuresMap = new ArrayList<String[]>();
		readInTreasuresAndRoomsAmount();
		ArrayList<Integer> scrambledInts = getScrambledInts(rooms);
		treasures = getTreasures(scrambledInts);
		
		//Once the system is set up the command sharing object and connect to server.
		command = new CommandObject();
		register(this.command);		
		r = new Connector("127.0.1.1", 6009, this.command);
		Thread r2 = new Thread(r);
		r2.start();

		//Wait for the server to start up before continuing.
		waitForServer(r);
		//Set the main ID for client to contact the server.
		r.setId("Hider");
	}

	public HashMap<Integer,String[]> getTreasures(ArrayList<Integer> scrambledInts) {
		HashMap<Integer,String[]> temp = new HashMap<Integer,String[]>();
		Random rand = new Random();
		for(int i = 0; i<rooms; i++) {
			int randomInt = rand.nextInt(rooms/2);
			temp.put(scrambledInts.get(i), treasuresMap.get(randomInt));
		}
		return temp;
	}
	//Function responsible for reading in the treasures list.
	public void readInTreasuresAndRoomsAmount() {
		try (BufferedReader br = new BufferedReader(new FileReader("treasures"))) {
		    String line = br.readLine();
		    rooms = Integer.parseInt(line);
		    while ((line = br.readLine()) != null) {
		       String[] temp = line.split(",");
		       treasuresMap.add(temp);
		    }
		} catch(FileNotFoundException e) {
			System.out.println("File not found.");
		} catch(IOException e) {
			System.out.println("IO Exception");
		}
	}

	public ArrayList<Integer> getScrambledInts(int rooms) {
		ArrayList<Integer> temp = new ArrayList<Integer>();
		for(int i=0;i<rooms;i++) {
			temp.add(i);
		}
		Collections.shuffle(temp);
		return temp;
	}
	//An empty blocking method that ensures that server has time to set up before continuing running asynchrously.
	public void waitForServer(Connector r) {
		while(!(r.isRunning())){
			System.out.print("");
		} //Wait for the server to start up before continuing.
	}


	//Simple method that sends a send robot message and asks the server to pass it through.
	public void sendTresureFirstProperty(Connector r, int room) {
		r.sendMessage("%%found Hider TabUI " + room + " " + room + " \"(colour " + treasures.get(room)[0] + ") (shape " + treasures.get(room)[1] + ")\"");
	}

	public void sendScore(int room, String treasure) {
		int score = 0;
		//Correct Identification
		if(treasure.contains(treasures.get(room)[3])) {
			System.out.println(treasures.get(room)[3]);
			score = Integer.parseInt(treasures.get(room)[2]);
		} else { //Bad identification
			System.out.println(treasure);
			System.out.println(treasures.get(room)[3]);
			score = 0 - Integer.parseInt(treasures.get(room)[2]);
		}
		r.sendMessage("%%score Hider TabUI ," + score);
	}

	public void sendPicture(int room) {
		r.sendMessage("%%image Hider TabUI " +room + " " + treasures.get(room)[4]);
	}

	//Listener functions implemented.
	public void register(Observable observable) {observable.add(this);}
  	public void unregister(Observable observable) {observable.remove(this);}

  	public void fieldChanged(Object source, String attribute) {

  		if(attribute.contains("error")) 
  		{
  			String[] temp = attribute.split("\"");
  			int room = Integer.parseInt(temp[1]);
  			sendTresureFirstProperty(r, room);
  		} 
  		else if(attribute.contains("found")) 
  		{
  			String[] temp = attribute.split("\"");
  			temp = temp[1].split(",");
  			int room = Integer.parseInt(temp[0]);
  			String treasure = temp[1];
  			System.out.println(room + " " + treasure);
  			sendScore(room,treasure);
  		} 
  		else if(attribute.contains("snap")) 
  		{
  			String[] temp = attribute.split(" ");
  			int room = Integer.parseInt(temp[temp.length-1]);
  			sendPicture(room);
  		}
    	System.out.println("Hider GUI: " + attribute); // this has to be implemented
  	}
}