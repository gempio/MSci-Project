import java.util.*;

public class GuiHider {

	static HashMap<Integer,String[]> treasures;

	public static void main(String[] args) {

		treasures = new HashMap<Integer,String[]>();
		String[] temp = {"Red","Ball"};
		treasures.put(0, temp);
		temp = {"Blue","Hat"};
		treasures.put(1, temp);
		temp = {"Red", "Square"};
		treasures.put(2, temp);
		temp = {"Green", "Hat"};
		treasures.put(3, temp);
		temp = {"Yellow","Bottle"};
		treasures.put(4, temp);
		temp = {"Orange", "Bottle"};
		treasures.put(5, temp);
		temp = {"Pink", "Hat"};
		treasures.put(6, temp);
		temp = {"Violet", "Square"};
		treasures.put(7, temp);


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


}