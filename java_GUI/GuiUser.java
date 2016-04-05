import java.util.Scanner;
import java.io.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.*;
/** 
	A class responsible for holding user data and listening to specific events
*/
public class GuiUser implements Listener{
	//Just a starter method for GUI with basic CLI.
	CommandObject command;
	Connector r;
	public static void main(String[] args) {
		new GuiUser();
	}

	public GuiUser() {
		boolean one_run = true;
		command = new CommandObject();
		addListener(command);
		this.r = new Connector("127.0.1.1", 6009,command);
		Thread r2 = new Thread(r);
		r2.start();
		waitForServer(r);
		r.setId("TabUI");
		new GUI();
		askQuestions(this.r);
		
	}

	public void addListener(CommandObject command) {
		command.add(this);
	}

	//An empty blocking method that ensures that server has time to set up before continuing running asynchrously.
	public static void waitForServer(Connector r) {
		while(!(r.isRunning())){
			System.out.print("");
		} //Wait for the server to start up before continuing.
	}

	//A simple CLI algorithm that keeps asking for where should the robot go and which.
	public void askQuestions(Connector r) {
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

	public void askForTreasure(int room ) {
		r.sendMessage("%%error TabUI Hider  \""+room+"\"");
	}

	public void register(Observable observable) {observable.add(this);}
  	public void unregister(Observable observable) {observable.remove(this);}

  	public void fieldChanged(Object source, String attribute) {
    	System.out.println("User GUI: " + attribute); 
    	if(attribute.contains("error")) {
    		String[] temp = attribute.split("\"");
    		temp = temp[1].split(";");
    		System.out.println("Robot: " + (temp[0]+1) + " Room: " + (temp[1]+1));
    		askForTreasure(Integer.parseInt(temp[1]));
    	}// this has to be implemented
    	if(attribute.contains("found")) System.out.println("Hider Message about the treasure");
  	}

  	private class GUI extends JFrame{

		public GUI() {
			JFrame frame = new JFrame("Frame Trial");
	        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	 
	        JLabel emptyLabel = new JLabel("");
	        emptyLabel.setPreferredSize(new Dimension(175, 100));
	        frame.getContentPane().add(emptyLabel, BorderLayout.CENTER);
	 
	        //Display the window.
	        frame.pack();
	        frame.setVisible(true);
		}

		public void buildRobotAskingPanel() {

		}

		public void buildMainPanel() {
			
		}

	}

}

