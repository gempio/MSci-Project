import java.util.*;
import java.io.*;

public class Dialogue {
	
	ArrayList<String> notSureQuestions;
	ArrayList<String> specificRoomQuestions;
	HashMap<Integer, Integer> roomTimes;
	int consensus; //Set to 0 when the dialogue is in progress, set to -1 when no consensus is reached and to 1 when it is.
	boolean typeOfDialogue; //If its false, we initialize notSure if its true we talk with specific room in mind.
	int curQuestion;
	int nextRoom;
	static int[][] mapCosts;

	public Dialogue() {
		notSureQuestions = new ArrayList<String>();
		specificRoomQuestions = new ArrayList<String>();
		consensus = 0;
		typeOfDialogue = false;
		curQuestion = 0;
		mapCosts = readInCosts();
		readInTheDialogue();
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
		consensus = 0;
		typeOfDialogue = false;
		curQuestion = 0;
	}

	//Number of questions for not sure.
	public void noQuestionsNS() {

	}

	//Number of questions for specific room
	public void noQuestionsSR() {

	}

	//Start a Don't know dialogue.
	public void startNotSure(int curRoom, ArrayList<Integer> unvisitedRooms) {
		int suggestedRoom = calculateSuggestion(curRoom, unvisitedRooms);
		restart();
		typeOfDialogue = true;
	}

	public String getNextQuestion() {
		curQuestion++;
		if(typeOfDialogue && curQuestion-1<notSureQuestions.size()) return notSureQuestions.get(curQuestion-1);
		else if(!typeOfDialogue && curQuestion-1<specificRoomQuestions.size()) return specificRoomQuestions.get(curQuestion-1);
		else return "End of Questions";
	}

	//Start a specific room selected dialogue.
	public void startSpecific(int cost, int room) {
		restart();
		typeOfDialogue = false;
	}

	public static int calculateSuggestion(int room, ArrayList<Integer> unvisitedRooms) {
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
	private static List<String> permutation(String prefix, String str) {
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

	public static void main(String[] args) {
		new Dialogue();
		ArrayList<Integer> unvisitedRooms = new ArrayList<Integer>();
		unvisitedRooms.add(1);
		unvisitedRooms.add(2);
		unvisitedRooms.add(3);
		System.out.println(calculateSuggestion(0, unvisitedRooms));
	}

}