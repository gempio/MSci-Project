import java.util.*;
import java.io.*;

public class Dialogue {
	
	private ArrayList<String> notSureQuestions;
	private ArrayList<String> specificRoomQuestions;
	private HashMap<Integer, Integer> roomTimes;
	private boolean typeOfDialogue; //If its false, we initialize notSure if its true we talk with specific room in mind.
	private int curQuestion;
	private int curRoom;
	private int nextRoom;
	private int suggestedRoom;
	private int[][] mapCosts;
	private boolean useShortPaths;
	private ArrayList<Integer> unvisitedRooms;

	public Dialogue(ArrayList<Integer> unvisitedRooms) {
		this.unvisitedRooms = unvisitedRooms;
		useShortPaths = true;
		notSureQuestions = new ArrayList<String>();
		specificRoomQuestions = new ArrayList<String>();
		typeOfDialogue = false;
		curQuestion = 0;
		mapCosts = readInCosts();
		readInTheDialogue();
		System.out.println(notSureQuestions.size());
	}

	public void setShortestPaths(boolean paths) {
		this.useShortPaths = paths;
	}
	public int getCost(int rooma, int roomb) {
		return mapCosts[rooma][roomb];
	}

	//Reads in the dialogue file.
	public void readInTheDialogue() {
		try (BufferedReader br = new BufferedReader(new FileReader("DialogueText"))) {
		    String line = br.readLine();
		    boolean nextSection = false;
		    while ((line = br.readLine()) != null) {
		       if(line.contains("SpecificRoom:")) {
		       		nextSection = true;
		       		continue;
		       }
		       if(!nextSection) notSureQuestions.add(line);
		       else specificRoomQuestions.add(line);
		    }
		} catch(FileNotFoundException e) {
			System.out.println("File not found.");
		} catch(IOException e) {
			System.out.println("IO Exception");
		}
	}
	//Restart the dialogue
	public void restart() {
		typeOfDialogue = false;
		curQuestion = 0;
	}

	//Number of questions for not sure.
	public int noQuestionsNS() {
		return notSureQuestions.size();
	}

	//Number of questions for specific room
	public int noQuestionsSR() {
		return specificRoomQuestions.size();
	}

	public int getSuggestion() {
		return suggestedRoom;
	}
	//Start a Don't know dialogue.
	public void startNotSure(int curRoom) {
		this.curRoom = curRoom;
		this.suggestedRoom = calculateSuggestion(curRoom);
		restart();
		typeOfDialogue = true;
	}

	//Iterator approach like next question architecture.
	public String getNextQuestion() {
		curQuestion++;
		if(typeOfDialogue && curQuestion-1<notSureQuestions.size()) {
			String temp = notSureQuestions.get(curQuestion-1);
			temp = temp.replace("£room£", Integer.toString(suggestedRoom));
			temp = temp.replace("£cost£", Integer.toString(mapCosts[curRoom][suggestedRoom]));
			return temp;
		}
		else if(!typeOfDialogue && curQuestion-1<specificRoomQuestions.size()) {
			String temp = specificRoomQuestions.get(curQuestion-1);
			temp = temp.replace("£room£", Integer.toString(nextRoom));
			temp = temp.replace("£cost£", Integer.toString(mapCosts[curRoom][nextRoom]));
			return temp;
		}
		else return "End of Questions";
	}

	//Start a specific room selected dialogue.
	public void startSpecific(int curRoom, int nextRoom) {
		restart();
		this.nextRoom = nextRoom;
		typeOfDialogue = false;
	}

	public int calculateSuggestion(int room) {
		String unvisitedRoomsInString = "";
		
		for(int uRoom: unvisitedRooms) {
			unvisitedRoomsInString += Integer.toString(uRoom);
		}
		
		//All the permutations of possible paths.
		List<String> list = permutation(Integer.toString(room), unvisitedRoomsInString);
		//Calculate the costs of each path permutation.
		HashMap<Integer, String> pathCostsPerPermutation = new HashMap<Integer, String>();
		for(int i = 0; i<list.size(); i++) {
			String path = list.get(i);
			int cost = 0;
			for(int j = 0; j<unvisitedRooms.size(); j++) {
				int from = Character.getNumericValue(path.charAt(j));
				int to = Character.getNumericValue(path.charAt(j+1));
				cost += mapCosts[from][to];
			}
			pathCostsPerPermutation.put(cost, path);
		}

		Integer minCostPath = Collections.min(pathCostsPerPermutation.keySet());
		return Character.getNumericValue(pathCostsPerPermutation.get(minCostPath).charAt(1));
	}

	public void setUnvisitedRooms(ArrayList<Integer> unvisitedRooms) {
		this.unvisitedRooms = unvisitedRooms;
	}

	public int[][] readInCosts() {
		int[][] adjacencyMatrix;
		try (BufferedReader br = new BufferedReader(new FileReader("mapTimes"))) {
		    String line = br.readLine();
		    int noNodes = Integer.parseInt(line);
		    adjacencyMatrix = new int[noNodes][noNodes];
		    //Read in the weights of the Nodes.
		    for(int i = 0; i<noNodes;i++) {
		    	line = br.readLine();
		    	int[] weights = convertStringToIntArray(line.split(" "));
		    	for(int j = 0; j<noNodes;j++) {
		    		adjacencyMatrix[i][j] = weights[j];
		    	}
		    }
		    return adjacencyMatrix;
		} catch(FileNotFoundException e) {
			System.out.println("File not found.");
		} catch(IOException e) {
			System.out.println("IO Exception");
		}

		return null;
	}

	public int[] convertStringToIntArray(String[] temp) {
		int[] temp2 = new int[temp.length];
		for(int i = 0; i<temp.length;i++) {
			temp2[i] = Integer.parseInt(temp[i]);
		}
		return temp2;
	}
	private List<String> permutation(String prefix, String str) {
	    List<String> permutations = new ArrayList<>();
	    int n = str.length();
	    if (n == 0) {
	        permutations.add(prefix);
	    }
	    else {
	        for (int i = 0; i < n; i++)
	            permutations.addAll(permutation(prefix + str.charAt(i), str.substring(0, i) + str.substring(i + 1, n)));
	    }
	    return permutations;
	}

}