import java.util.Scanner;
import java.util.concurrent.LinkedBlockingQueue;
import java.io.*;

/** 
	A Cli class that manages sending robots around and contacting the server as TabUI.
*/
public class GuiUser {
	static LinkedBlockingQueue<String> commands;
	//Just a starter method for GUI with basic CLI.
	public static void main(String[] args) {
		commands = new LinkedBlockingQueue<String>();
		boolean one_run = true;
		Connector r = new Connector("127.0.1.1", 6009, commands);
		Thread r2 = new Thread(r);
		r2.start();
		waitForServer(r);
		r.setId("TabUI");
		try {
			//System.out.println(commands.take());
			commands.take();
			
		} catch(InterruptedException e) {
			System.out.println("InterruptedException");
		}
		askQuestions(r);
	}

	//An empty blocking method that ensures that server has time to set up before continuing running asynchrously.
	public static void waitForServer(Connector r) {
		while(!(r.isRunning())){
			System.out.print("");
		} //Wait for the server to start up before continuing.
	}

	//A simple CLI algorithm that keeps asking for where should the robot go and which.
	public static void askQuestions(Connector r) {
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