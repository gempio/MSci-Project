import java.util.*;
import java.io.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.*;
import javax.swing.table.*;
import java.awt.image.BufferedImage;
import javax.imageio.ImageIO;
import javax.swing.event.*;
import java.beans.*;
/** 
	A class responsible for holding user data and listening to specific events
*/
public class GuiUser implements Listener{
	//Just a starter method for GUI with basic CLI.
	CommandObject command;
	Connector r;
	Robot[] robots;
	JComboBox<String> noRobotsBox;
	int i;
	JButton[] rooms;
	int selectedRobot;
	int firstSelection;
	JTable table;
	GUI gui;
	JPanel mapPanel;
	JLabel map;
	DefaultTableModel tmodel;
	ArrayList<Integer> unvisitedRooms;
	Dialogue dialogue;
	int takePictureCost;
	int grabTreasureCost;
	boolean consensus;
	int curQuestion;
	int score;
	ArrayList<String[]> treasureOptions;
	String[] actualTreasOptions;
	JLabel scoreLabel;

	//Starter method
	public static void main(String[] args) {
		new GuiUser();
	}
	//Constructor for all the basic commands and server initialization.
	public GuiUser() {
		score = 0;
		firstSelection = 0;
		readInTreasuresAndRoomsAmount();
		command = new CommandObject();
		addListener(command);
		this.r = new Connector("127.0.1.1", 6009,command);
		Thread r2 = new Thread(r);
		r2.start();
		waitForServer(r);
		r.setId("TabUI");
		gui = new GUI();
		
	}

	public void addListener(CommandObject command) {
		command.add(this);
	}

	//An empty blocking method that ensures that server has time to set up before continuing running asynchronously.
	public static void waitForServer(Connector r) {
		while(!(r.isRunning())){
			System.out.print("");
		} //Wait for the server to start up before continuing.
	}

	//Simple method that sends a send robot message and asks the server to pass it through.
	public void sendRobot(Connector r, int robot, int room) {
		int costOfTravel = dialogue.getCost(robots[robot].getRoomNumber(), room+1);
		robots[robot].setCost(costOfTravel);
		robots[robot].setTraveling(true);
		r.sendMessage("%%goto TabUI SimR " + robot + " " + room + " " + room + " 45");
		unvisitedRooms.remove(new Integer(room+1));
		dialogue.setUnvisitedRooms(unvisitedRooms);
	}

	public void checkIfGameFinished() {
		int blockedRobots = 0;
		for(int i= 0; i<robots.length;i++) {
			if(robots[i].getBlocked()) blockedRobots++;
		}
		if(blockedRobots == robots.length) gui.showFinishSplashDialog();
	}

	public void readInTreasuresAndRoomsAmount() {
		ArrayList<String> tempTreasArray = new ArrayList<String>();
		treasureOptions = new ArrayList<String[]>();
		try (BufferedReader br = new BufferedReader(new FileReader("treasures"))) {
		    String line = br.readLine();
		    while ((line = br.readLine()) != null) {
		       String[] temp = line.split(",");
		       tempTreasArray.add(temp[3]);
		       treasureOptions.add(temp);
		    }
		} catch(FileNotFoundException e) {
			System.out.println("File not found.");
		} catch(IOException e) {
			System.out.println("IO Exception");
		}
		actualTreasOptions = new String[treasureOptions.size()];
		actualTreasOptions = tempTreasArray.toArray(actualTreasOptions);
	}

	public void askForTreasure(int room ) {
		r.sendMessage("%%error TabUI Hider  \""+room+"\"");
	}

	public void register(Observable observable) {observable.add(this);}
  	public void unregister(Observable observable) {observable.remove(this);}

  	//Calculate all the new attributes of the robot.
  	public void fieldChanged(Object source, String attribute) {

    	if(attribute.contains("error")) {
    		updateRobot(attribute);
    	}// this has to be implemented
    	else if(attribute.contains("found")) {
    		gui.createDialogs(attribute);
    	} else if(attribute.contains("score")) {
    		String recievedPoints = attribute.split(",")[1];
    		recievedPoints = recievedPoints.substring(0,recievedPoints.length()-1);
    		gui.showMessage(Integer.parseInt(recievedPoints));
    		
    		score += Integer.parseInt(recievedPoints);
    		scoreLabel.setText("Score: " + score);
    	} else if(attribute.contains("image")) {
    		String[] brokenDownCommand = attribute.split(" ");
    		String filename = brokenDownCommand[brokenDownCommand.length-1];
    		int room = Integer.parseInt(brokenDownCommand[brokenDownCommand.length-2]);
    		gui.takePicture(filename,room);
    	}
  	}

  	//Update robot values after it reached its goal.
  	public void updateRobot(String attribute) {

  		//Split up the message
  		String[] temp = attribute.split("\"");
		temp = temp[1].split(";");

		//Send a message regarding the treasure
		askForTreasure(Integer.parseInt(temp[1]));

		//Update the robot information
		int robotId = Integer.parseInt(temp[0]);
		robots[robotId].setLocation(Integer.parseInt(temp[1])+1);
		robots[robotId].setTraveling(false);
		robots[robotId].subtractCost();

		//Change the maps and update the table
		if(selectedRobot == robotId) gui.updateMap(robots[robotId].getRoomNumber());
		tmodel.fireTableDataChanged();
		table.changeSelection(selectedRobot, 0, false, false);
  	}
  	//Class responsible for creating the GUI and handling its logic.
  	private class GUI extends JFrame{
  		
  		private static final long serialVersionUID = 1L;

		public GUI() {
			super("Treasure Hunt");

			unvisitedRooms = new ArrayList<Integer>();
			for(int i = 1; i<9; i++) {
				unvisitedRooms.add(i);
			}
			dialogue = new Dialogue(unvisitedRooms);
	        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	 
	        JLabel emptyLabel = new JLabel("");
	        emptyLabel.setPreferredSize(new Dimension(175, 100));
	        this.getContentPane().add(emptyLabel, BorderLayout.CENTER);
	 
	        //Display the window.
	        scoreLabel = new JLabel("Score: " + score);
	        buildRobotAskingPanel();
	        this.pack();
	        this.setVisible(true);
		}

		//This message tells the user whether or not their identification was correct.
		public void showMessage(int score) {
			if(score>0) {
    			JOptionPane.showMessageDialog(this, "Identification Successful: " + score + " points", "Success", JOptionPane.INFORMATION_MESSAGE);
    		} else {
    			JOptionPane.showMessageDialog(this, "Identification Unsuccessful: " + score + " points", "Attempt Failed", JOptionPane.INFORMATION_MESSAGE);
    		}
		}
		public void buildRobotAskingPanel() {
			JPanel container = new JPanel();
			container.setLayout(new BoxLayout(container, BoxLayout.PAGE_AXIS));
			JButton continueBtn = new JButton("Continue");
			String[] noRobots = { "1","2"};

			noRobotsBox = new JComboBox<String>(noRobots);
			noRobotsBox.setSelectedIndex(0);

			continueBtn.addActionListener(new ActionListener(){

				public void actionPerformed(ActionEvent e)
	            {
	                //Execute when button is pressed
	            	String temp = String.valueOf(noRobotsBox.getSelectedItem());
	            	robots = new Robot[Integer.parseInt(temp)];
	                buildMainPanel();
	            }


			});
			
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

			//Fill out the container.
			container.add(listPane, BorderLayout.CENTER);
			container.add(buttonPane, BorderLayout.PAGE_END);
			this.add(container);
		}

		public void buildMainPanel() {
			//Create a new container
			JPanel container = new JPanel();
			
			//Let's start the robots off.
			for(int i=0; i<robots.length; i++) {
				robots[i] = new Robot("Robot " + (i+1), i, 100, "-1");
			}

			//Introduce the main panels.
			JPanel topLabelPanel = new JPanel(new BorderLayout());
			//topLabelPanel.setBackground(Color.BLACK);
			mapPanel = new JPanel();
			//mapPanel.setBackground(Color.BLUE);
			JPanel robotList = new JPanel();
			//robotList.setBackground(Color.GREEN);
			JPanel roomOptionList = new JPanel();
			//roomOptionList.setBackground(Color.WHITE);
			JPanel middlePanel = new JPanel();
			JPanel bottomPanel = new JPanel();

			//Set their sizes.
			topLabelPanel.setPreferredSize(new Dimension(1000,20));
			mapPanel.setPreferredSize(new Dimension(800,600));
			robotList.setPreferredSize(new Dimension(180,600));
			roomOptionList.setPreferredSize(new Dimension(800,100));

			tmodel = new DefaultTableModel();
			tmodel.addColumn("NoHeader", robots);
			//Fill out the robotList
			table = new JTable(tmodel) {
				 private static final long serialVersionUID = 2L;
				 public boolean isCellEditable(int row, int column) {return false;}
			};
    		table.setDefaultRenderer(Object.class, new RobotRenderer());
    		ListSelectionListener cellsChange = new ListSelectionListener() {
    			public void valueChanged(ListSelectionEvent e) {
    				if (e.getValueIsAdjusting()) return; 
    				int temp = ((DefaultListSelectionModel) e.getSource()).getMinSelectionIndex();
    				if(temp != -1) {
    					selectedRobot = temp;
    					firstSelection = temp;
    				}
    				else selectedRobot = firstSelection;
    				updateButtons(robots[selectedRobot]);
    				updateMap(robots[selectedRobot].getRoomNumber());
    			}
    		};

    		table.getSelectionModel().addListSelectionListener(cellsChange);
    		table.setTableHeader(null);
    		table.setRowHeight(90);
    		robotList.setLayout(new BorderLayout());  
    		robotList.add(new JScrollPane(table));   

			//Fill out the top panel
			topLabelPanel.add(new JLabel("Maximize the score by identifying correct treasures"), BorderLayout.WEST);
			topLabelPanel.add(scoreLabel, BorderLayout.EAST);
			topLabelPanel.setBorder(BorderFactory.createEmptyBorder(0, 10, 10, 10));

			//Fill out the first basic map.
			map = getImageLabel("maps/mapR1.png");
			mapPanel.add(map);

			//Fill out the middle panel
			middlePanel.setLayout(new BorderLayout());
			middlePanel.add(mapPanel,BorderLayout.WEST);
			middlePanel.add(robotList,BorderLayout.EAST);

			//Fill out the bottom panel
			bottomPanel.setLayout(new BorderLayout());
			bottomPanel.add(Box.createRigidArea(new Dimension(0,5)), BorderLayout.NORTH);
			bottomPanel.add(new JLabel("Please select a room to go to: "), BorderLayout.NORTH);
			
			//Populate room buttons
			roomOptionList.setLayout(new FlowLayout());
			int noRooms = 8;
			rooms = new JButton[noRooms+1];
			rooms[0] = new JButton("Don't know");
			rooms[0].addActionListener(new ActionListener() {
				public void actionPerformed(ActionEvent e) {
					initializeDialogue(false, robots[selectedRobot].getRoomNumber(), 0);	
					if(consensus) {
						int suggestion = dialogue.getSuggestion();
						sendRobot(r, selectedRobot,suggestion-1);
						robots[selectedRobot].setTraveling(true);
						tmodel.fireTableDataChanged();
		            	blockButtons();
					}
				}
				
			});
			roomOptionList.add(rooms[0]);
			for(i = 1; i<noRooms+1; i++) {
				rooms[i] = new JButton("" + (i));
				rooms[i].setPreferredSize(new Dimension(60,60));
				rooms[i].addActionListener(new ActionListener(){

					public void actionPerformed(ActionEvent e)
	            	{
		                //Execute when button is pressed
		                String command = ((JButton) e.getSource()).getActionCommand();
		            	//Initialize Specific room dialogue this will return true or false depending on consensus.
		            	initializeDialogue(true, robots[selectedRobot].getRoomNumber(), Integer.parseInt(command));
		            	//If consensus was reached, send the robot.
		            	if(consensus) {
		            		sendRobot(r, selectedRobot,Integer.parseInt(command)-1);
		            		robots[selectedRobot].setTraveling(true);
		            		blockButtons();
		            	}
	            	}

				});
				roomOptionList.add(rooms[i]);
			}

			bottomPanel.add(roomOptionList, BorderLayout.CENTER);
			//Fill out the main container
			container.setLayout(new BorderLayout());
			container.add(topLabelPanel, BorderLayout.NORTH);
			container.add(middlePanel, BorderLayout.CENTER);
			container.add(bottomPanel, BorderLayout.SOUTH);


			//Set the new container and repaint.
			table.changeSelection(0, 0, false, false);
			this.setContentPane(container);
			this.validate();
			this.pack();
			this.repaint();
		}

		public JLabel getImageLabel(String path) {
			try{ 
				BufferedImage myPicture = ImageIO.read(new File(path));
				JLabel picLabel = new JLabel(new ImageIcon(myPicture));
				return picLabel;
			} catch(IOException e){
				return null;
			}
		}

		public void updateMap(int roomId) {
			System.out.println(roomId);
			System.out.println("maps/mapR"+(roomId)+".png");
			
			try{ 
				BufferedImage myPicture = ImageIO.read(new File("maps/mapR"+(roomId)+".png"));
				map.setIcon(new ImageIcon(myPicture));
			} catch(IOException e){
				System.out.println("Image not loaded");
			}

			mapPanel.repaint();
		}

		public void createDialogs(String attributes) {
			//Take in the information regarding the treasure and who it.
			String[] temp = attributes.split("\\(");
			attributes = temp[1].substring(0,temp[1].length()-2) + " " + temp[2].substring(0,temp[2].length()-2);
			String[] optionsForTreasure= {"Take Picture", "Identify the Treasure", "Continue"};
			final JOptionPane optionPane = new JOptionPane(
                                    attributes,
                                    JOptionPane.QUESTION_MESSAGE,
                                    JOptionPane.YES_NO_CANCEL_OPTION, null);

			optionPane.setOptions(optionsForTreasure);
			optionPane.setSelectionValues(actualTreasOptions);
            final JDialog dialog = new JDialog(this,
                                         "Identified a shape",
                                         true);
            dialog.setContentPane(optionPane);
            dialog.setDefaultCloseOperation(
                JDialog.DO_NOTHING_ON_CLOSE);
            optionPane.addPropertyChangeListener(
                new PropertyChangeListener() {
                    public void propertyChange(PropertyChangeEvent e) {
                        String prop = e.getPropertyName();

                        if (dialog.isVisible()
                         && (e.getSource() == optionPane)
                         && (JOptionPane.VALUE_PROPERTY.equals(prop))) {
                            dialog.setVisible(false);
                        }
                    }
                });
            dialog.pack();
            dialog.setLocationRelativeTo(this);
            dialog.setVisible(true);

            String value = (String) optionPane.getValue();
            int room = Integer.parseInt((temp[0].split(" "))[4]);
            if (value.equals("Take Picture")) {
                r.sendMessage("%%snap TabUI Hider " + room);
            } else if (value.equals("Identify the Treasure")) {
            	String identification = (String) optionPane.getInputValue();
                sendIdentification(room, identification);
            }
		}

		public void sendIdentification(int room, String identification) {
			r.sendMessage("%%found TabUI Hider \"" + room+","+identification);
		}

		public void takePicture(String filename, int room) {
			//Pretty much creating the same type of dialog as in asking to take picture but setting text Icon to the corresponding image
			 try
                {
                	String[] optionsForTreasure = {"Identify","Continue"};
                	final JOptionPane optionPane = new JOptionPane(
                                    optionsForTreasure,
                                    JOptionPane.QUESTION_MESSAGE,
                                    JOptionPane.YES_NO_OPTION, null);

                	optionPane.setOptions(optionsForTreasure);
					optionPane.setSelectionValues(actualTreasOptions);

					

                    JDialog dialog = new JDialog();
                    dialog.setLayout(new BorderLayout());
                    dialog.setDefaultCloseOperation(JDialog.DO_NOTHING_ON_CLOSE);
                    dialog.setTitle("Picture of treasure");
                    dialog.add(new JLabel(new ImageIcon(ImageIO.read(getClass().getResourceAsStream("rooms/" + filename)))),BorderLayout.NORTH);
                    dialog.add(optionPane,BorderLayout.SOUTH);

                    optionPane.addPropertyChangeListener(new PropertyChangeListener() {
	                    
	                    public void propertyChange(PropertyChangeEvent e) {
	                        String prop = e.getPropertyName();

	                        if (dialog.isVisible() && (e.getSource() == optionPane)&& (JOptionPane.VALUE_PROPERTY.equals(prop))) {
	                         	String value = (String) optionPane.getValue();
					            if (value.equals("Identify")) {
					            	String identification = (String) optionPane.getInputValue();
					                sendIdentification(room, identification);
					            } else if (value.equals("Continue")) {
					            	dialog.setVisible(false);
					            }
	                            dialog.setVisible(false);
	                        }
	                    }
	                });

                    dialog.pack();
                    dialog.setLocationByPlatform(true);
                    dialog.setVisible(true);   
                } 
                catch (IOException e) 
                {
                    e.printStackTrace();
                }

		}
		public void blockButtons() {
			for(i=0; i<rooms.length; i++) {
				rooms[i].setEnabled(false);
			}
		}

		public void updateButtons(Robot robot) {

			if(robot.isTraveling()) {
				for(i=1; i<rooms.length; i++) {
					rooms[i].setEnabled(false);
				}
			} else {
				//If the robot is blocked, no point in updating its buttons.
				if(robot.getBlocked()) return;
				rooms[0].setEnabled(true);
				for(i=1; i<rooms.length; i++) {
					if(unvisitedRooms.contains(i)) rooms[i].setEnabled(true);
				}
			}

		}

		public boolean initializeDialogue(boolean specific, int start, int end) {
			int questionsNo = 0;
			if(specific) {
				dialogue.startSpecific(start, end);
				questionsNo = dialogue.noQuestionsSR();
			}
			else {
				dialogue.startNotSure(start);
				questionsNo = dialogue.noQuestionsNS();
			}
			int questions = questionsNo;
			if(questions == 0) {
				consensus = true;
				return true;
			}
			curQuestion = 1;
			String question = dialogue.getNextQuestion();

			String[] optionsForConsensus= {"Continue", "Stop"};
			final JOptionPane optionPane = new JOptionPane(
                                    question,
                                    JOptionPane.QUESTION_MESSAGE,
                                    JOptionPane.YES_NO_OPTION);

			optionPane.setOptions(optionsForConsensus);
            final JDialog dialog = new JDialog(this,
                                         "Dialogue",
                                         true);
            dialog.setContentPane(optionPane);
            dialog.setDefaultCloseOperation(
                JDialog.DO_NOTHING_ON_CLOSE);
            dialog.addWindowListener(new WindowAdapter() {
                public void windowClosing(WindowEvent we) {
                    dialog.setVisible(false);
                }
            });
            optionPane.addPropertyChangeListener(
                new PropertyChangeListener() {
                    public void propertyChange(PropertyChangeEvent e) {
                        String prop = e.getPropertyName();
                        if (dialog.isVisible() && (e.getSource() == optionPane) && (JOptionPane.VALUE_PROPERTY.equals(prop))) {
                        	String value = (String) optionPane.getValue();
                        	optionPane.setValue("Something");

                        	if (value.equals("Continue")) {
			                optionPane.setMessage(dialogue.getNextQuestion());
			                dialog.repaint();
			                if(curQuestion == questions) {
			                	dialog.setVisible(false);
			                	consensus = true;
			                }
			                curQuestion++;
			            	} else if (value.equals("Stop")) {
			            		dialog.setVisible(false);
			            		consensus = false;
			            	}
                        }
                    }
                });
            dialog.setPreferredSize(new Dimension(600,150));
            dialog.setModal(true);
            dialog.pack();
            dialog.setLocationRelativeTo(this);
            dialog.setVisible(true);
            return consensus;

		}
		public void showFinishSplashDialog(boolean cleanFinish) {
			if(!cleanFinish) JOptionPane.showMessageDialog(this, "You have visited all rooms. You have finished the game with a score of: " + score + ". \\nCongratulations", "Game over.", JOptionPane.INFORMATION_MESSAGE);
			else JOptionPane.showMessageDialog(this, "Both robots have ran out of energy. You have finished the game with a score of: " + score + ". \\nCongratulations", "Game over.", JOptionPane.INFORMATION_MESSAGE);
		}

	}



}