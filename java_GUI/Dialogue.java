import java.util.*;
import java.io.*;

public class Dialogue {
	
	ArrayList<String> notSureQuestions;
	ArrayList<String> specificRoomQuestions;
	HashMap<Integer, Integer> roomTimes;
	int consensus; //Set to 0 when the dialogue is in progress, set to -1 when no consensus is reached and to 1 when it is.
	boolean typeOfDialogue; //If its false, we initialize notSure if its true we talk with specific room in mind.
	int curQuestion;

	public Dialogue() {
		notSureQuestions = new ArrayList<String>();
		specificRoomQuestions = new ArrayList<String>();
		consensus = 0;
		typeOfDialogue = false;
		curQuestion = 0;
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
	public void startNotSure(Robot robot) {
		int suggestedRoom = calculateSuggestion();
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

	public int calculateSuggestion() {
		return 0;
	}

	public static void main(String[] args) {
		new Dialogue();
	}

}