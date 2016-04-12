import java.io.*;
import java.net.*;
import java.util.concurrent.LinkedBlockingQueue;

/*
	Connector class is responsible for running the server. Listening to basic information coming in to the classes that
	use it and moving forward.

*/
public class Connector implements Runnable{
	Socket kkSocket;
	PrintWriter out;
	BufferedReader in;
	String hostName;
	int portNumber;
	boolean isAble;
	boolean sendMessage;
	String fromUser;
	CommandObject commands;

	public Connector(String hostName, int portNumber, CommandObject commands) {
		this.hostName = hostName;
		this.portNumber = portNumber;
		this.isAble = false;
		this.fromUser = "";
		this.sendMessage = false;
		this.commands = commands;
	}

	public void sendMessage(String message) {
		System.out.println("Message Sending: " + message);
		out.println(message);
	}

	public void setId(String id) {
		out.println("%%setid " + id);
	}

	public boolean isRunning() {
		if(isAble) return true;
		else return false;
	}
	public void makeOutPublic(PrintWriter out) {
		this.out = out;
	}

	public void run() {
 		
        try (
            Socket kkSocket = new Socket(hostName, portNumber);
            PrintWriter out = new PrintWriter(kkSocket.getOutputStream(), true);
            BufferedReader in = new BufferedReader(new InputStreamReader(kkSocket.getInputStream()));
        ) {
            BufferedReader stdIn = new BufferedReader(new InputStreamReader(System.in));
            String fromServer;
            
 			makeOutPublic(out);
 			isAble = true;
            while ((fromServer = in.readLine()) != null) {
                System.out.println("Server: " + fromServer);
                if (fromServer.contains("ping")) out.println("%%pong");
                else if (fromServer.contains("error") || fromServer.contains("found")) commands.setField(fromServer);
                else if (fromServer.contains("ack")) System.out.println("ackowledged");
            }
        } catch (UnknownHostException e) {
            System.err.println("Don't know about host " + hostName);
            System.exit(1);
        } catch (IOException e) {
            System.err.println("Couldn't get I/O for the connection to " +
                hostName);
            System.exit(1);
        }

	}


	
}