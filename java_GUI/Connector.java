
import java.io.*;
import java.net.*;

public class Connector implements Runnable{
	Socket kkSocket;
	PrintWriter out;
	BufferedReader in;
	String hostName;
	int portNumber;
	boolean isAble;
	boolean sendMessage;
	String fromUser;

	public Connector(String hostName, int portNumber) {
		this.hostName = hostName;
		this.portNumber = portNumber;
		isAble = false;
		fromUser = "";
		sendMessage = false;
	}

	public void sendRobot(int robot, int room) {
		System.out.println("%%goto TabUI SimR " + robot + " " + room + " " + room + " 45");
		out.println("%%goto TabUI SimR " + robot + " " + room + " " + room + " 45");
		sendMessage = true;
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
            while ((fromServer = in.readLine()) != null || sendMessage) {
                System.out.println("Server: " + fromServer);
                if (fromServer.contains("ping"))
                    out.println("%%pong");
                else if (fromServer.contains("ack")) {
                    System.out.println("ackowledged");
                }

                if(sendMessage) {
                	System.out.println("Sending Message");
                	out.println(fromUser);
                	sendMessage = false;
                }
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