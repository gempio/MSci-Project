
import java.io.*;
import java.net.*;
import java.lang.Exception;

public class Listener implements Runnable {
	
	Socket kkSocket;
	PrintWriter out;
	BufferedReader in;

	public Listener(Socket kkSocket, PrintWriter out, BufferedReader in) {
		this.kkSocket = kkSocket;
		this.out = out;
		this.in = in;

	}

	public void run() {
		String fromServer;
		try {
			while ((fromServer = in.readLine()) != null) {
		        System.out.println("Server: " + fromServer);
		        if (fromServer.contains("ping"))
		            out.println("%%pong");
		        else if (fromServer.contains("ack")) {
		            System.out.println("ackowledged");
	    		}

			}
		} catch(IOException e) {
			System.err.println("Caught IOException: " + e.getMessage());
		}
	}
}