import java.util.Scanner;
import java.io.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.*;
import javax.swing.table.DefaultTableModel;
import javax.swing.table.TableCellRenderer;
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
			super("Treasure Hunt");
	        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	 
	        JLabel emptyLabel = new JLabel("");
	        emptyLabel.setPreferredSize(new Dimension(175, 100));
	        this.getContentPane().add(emptyLabel, BorderLayout.CENTER);
	 
	        //Display the window.
	        buildRobotAskingPanel();
	        this.pack();
	        this.setVisible(true);
		}

		public void buildRobotAskingPanel() {
			System.out.println("Robot Number Panel Called");
			JPanel container = new JPanel();
			container.setLayout(new BoxLayout(container, BoxLayout.PAGE_AXIS));
			JButton continueBtn = new JButton("Continue");
			continueBtn.addActionListener(new ActionListener(){

				public void actionPerformed(ActionEvent e)
	            {
	                //Execute when button is pressed
	                buildMainPanel();
	            }


			});
			String[] noRobots = { "1","2"};

			//Create the combo box, select item at index 4.
			//Indices start at 0, so 4 specifies the pig.
			JComboBox<String> noRobotsBox = new JComboBox<String>(noRobots);
			noRobotsBox.setSelectedIndex(0);

			//Code to EDDDITTTTTTTTTTTTTTTTTTTTTT
			//Lay out the label and scroll pane from top to bottom.
			JPanel listPane = new JPanel();
			listPane.setLayout(new BoxLayout(listPane, BoxLayout.PAGE_AXIS));
			JLabel label = new JLabel("Please provide number of robots");
			listPane.add(label);
			listPane.add(Box.createRigidArea(new Dimension(0,5)));
			listPane.add(noRobotsBox);
			listPane.setBorder(BorderFactory.createEmptyBorder(10,10,10,10));

			//Lay out the buttons from left to right.
			JPanel buttonPane = new JPanel();
			buttonPane.setLayout(new BoxLayout(buttonPane, BoxLayout.LINE_AXIS));
			buttonPane.setBorder(BorderFactory.createEmptyBorder(0, 10, 10, 10));
			buttonPane.add(Box.createHorizontalGlue());
			buttonPane.add(continueBtn);


			container.add(listPane, BorderLayout.CENTER);
			container.add(buttonPane, BorderLayout.PAGE_END);
			this.add(container);
		}

		public void buildMainPanel() {
			//Create a new container
			JPanel container = new JPanel();
			
			//Fill out the new container

			//Introduce the main panels.
			JPanel topLabelPanel = new JPanel();
			topLabelPanel.setBackground(Color.BLACK);
			JPanel mapPanel = new JPanel();
			mapPanel.setBackground(Color.BLUE);
			JPanel robotList = new JPanel();
			robotList.setBackground(Color.GREEN);
			JPanel roomOptionList = new JPanel();
			roomOptionList.setBackground(Color.WHITE);
			JPanel middlePanel = new JPanel();
			JPanel bottomPanel = new JPanel();

			//Set their sizes.
			topLabelPanel.setPreferredSize(new Dimension(1000,20));
			mapPanel.setPreferredSize(new Dimension(800,600));
			robotList.setPreferredSize(new Dimension(180,600));
			roomOptionList.setPreferredSize(new Dimension(800,100));


			DefaultTableModel tmodel = new DefaultTableModel();
      		DefaultTableModel model = new DefaultTableModel();
			tmodel.addColumn("NoHeader",new Object[] { new Robot("Robot 1",0,100),
      		new Robot("Robot 2",2,100)});
			//Fill out the robotList
			JTable table = new JTable(tmodel);
    		table.setCellEditor(new RobotRenderer());
    		table.setTableHeader(null);
    		robotList.setLayout(new BorderLayout());  
    		robotList.add(new JScrollPane(table));   

			//Fill out the top panel
			topLabelPanel.add(new JLabel("Objectives can be found below"));

			//Fill out the middle panel
			middlePanel.setLayout(new BorderLayout());
			middlePanel.add(mapPanel,BorderLayout.WEST);
			middlePanel.add(robotList,BorderLayout.EAST);

			//Fill out the bottom panel
			bottomPanel.setLayout(new BorderLayout());
			bottomPanel.add(Box.createRigidArea(new Dimension(0,5)), BorderLayout.NORTH);
			bottomPanel.add(new JLabel("Please select a room to go to"), BorderLayout.NORTH);
			bottomPanel.add(roomOptionList, BorderLayout.CENTER);

			//Fill out the main container
			container.setLayout(new BorderLayout());
			container.add(topLabelPanel, BorderLayout.NORTH);
			container.add(middlePanel, BorderLayout.CENTER);
			container.add(bottomPanel, BorderLayout.SOUTH);


			//Set the new container and repaint.
			this.setContentPane(container);
			this.validate();
			this.pack();
			this.repaint();
		}

	}

}

