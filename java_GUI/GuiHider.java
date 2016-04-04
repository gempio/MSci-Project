import java.util.*;

public class GuiHider {

	static HashMap<Integer,String[]> treasures;
	
	public static void main(String[] args) {

		treasures = new HashMap<Integer,String[]>();
		
		treasures.put(0, setTreasure("Red","Ball"));
		treasures.put(1, setTreasure("Blue","Hat"));
		treasures.put(2, setTreasure("Red", "Square"));
		treasures.put(3, setTreasure("Green", "Hat"));
		treasures.put(4, setTreasure("Yellow","Bottle"));
		treasures.put(5, setTreasure("Orange", "Bottle"));
		treasures.put(6, setTreasure("Pink", "Hat"));
		treasures.put(7, setTreasure("Violet", "Square"));


		Connector r = new Connector("127.0.1.1", 6009);
		Thread r2 = new Thread(r);
		r2.start();
		
		while(!(r.isRunning())){
			System.out.print("");
		} //Wait for the server to start up before continuing.

		r.setId("Hider");

		while(true) {
			System.out.println("Enter your robot and room to go to e.g. 0;0: ");
			Scanner scanner = new Scanner(System.in);
			String sendRobotTo = scanner.nextLine();
			int room = Integer.parseInt(sendRobotTo.split(";")[0]);
			int robot = Integer.parseInt(sendRobotTo.split(";")[1]);
			System.out.println(room + " " + robot);
			
			sendRobot(r, room,robot);
		}

	}

	//Simple method that sends a send robot message and asks the server to pass it through.
	public static void sendRobot(Connector r, int robot, int room) {
		r.sendMessage("%%goto TabUI SimR " + robot + " " + room + " " + room + " 45");
	}
	//A method that returns an array of two property treasure.
	public static String[] setTreasure(String colour, String shape) {
		String[] temp = {colour,shape};
		return temp;
	}
}