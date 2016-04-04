import java.util.Scanner;

public class GuiUser {

	//Just a starter method for GUI with basic CLI.
	public static void main(String[] args) {
		boolean one_run = true;
		Connector r = new Connector("127.0.1.1", 6009);
		Thread r2 = new Thread(r);
		r2.start();
		waitForServer(r);
		askQuestions(r);
		r.setId("TabUI");
		
	}


	public static void waitForServer(Connector r) {
		while(!(r.isRunning())){
			System.out.print("");
		} //Wait for the server to start up before continuing.
	}

	public static void askQuestions(Connector r) {
		while(true) {
			System.out.println("Enter your robot and room to go to e.g. 0;0: ");
			Scanner scanner = new Scanner(System.in);
			String sendRobotTo = scanner.nextLine();
			int room = Integer.parseInt(sendRobotTo.split(";")[0]);
			int robot = Integer.parseInt(sendRobotTo.split(";")[1]);
			System.out.println(room + " " + robot);
			r.sendRobot(room,robot);
		}
	}

	public static void sendRobot(Connector r) {
		
	}
}