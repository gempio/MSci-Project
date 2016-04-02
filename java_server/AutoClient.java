/**
 * AutoClient.java
 *
 * A subclass of Client that sends regular messages to test the
 * infrastructure
 *
 * @author  Elizabeth Sklar
 * @author  Simon Parsons
 * @version 09-jul-1998 (original)
 * @version 28-feb-2015 (revised)
 * @version 17-mar-2015/sklar: added new commands (WHERE, HIDE, FOUND)
 * and removed obselete TREASURE command
 *
 */
import java.io.*;
import java.net.*;
import java.util.*;

public class AutoClient extends Client {

    Random rand    = new Random(System.currentTimeMillis());
    int    counter = 1;

    String destination;

    /**
     * Client constructor
     *
     */
    public AutoClient( String host, int port, String ID ) throws IOException {
	
	super( host, port, ID );

    } // end of Client constructor


    /**
     * main()
     *
     * This is the main method for this sample Client program. This
     * program asks a user to enter input, and then sends that input
     * to the Server. Uses a ServerThread object to do the heavy
     * lifting.
     *
     */
    public static void main( String[] args ) {

	AutoClient client;

	try {
	    // get the hostname and port from the command line. 
	    // fatal error if these are not provided. 
	    String host = args[0];
	    int    port = Integer.parseInt( args[1] );
	    String ID   = args[2];
	    String dest = args[3];

	    // instantiate new Client object
	    client = new AutoClient( host, port, ID );
	    client.setDestination( dest );

	    // start up main run thread
	    client.run();
	}
	catch ( IOException iox ) {
	    System.err.println( iox );
	    System.exit( 1 );
	}
	catch( ArrayIndexOutOfBoundsException aioobe ){
	    System.err.println( "usage: java AutoClient <host> <port> <myID> <destinationID>" );
	    System.exit( 1 );
	}	

    } // end of main()

    /**
     * run()
     *
     */
    public void run() {

	// Wait for server to set me up
	try {
	    Thread.sleep(5000);                 //take 5
	}
	catch( InterruptedException ex ) {
	    Thread.currentThread().interrupt();
	}

	// Ping server
	System.out.println( (new Date().getTime()) + " C: pinging server" );		
	thread.sendToServer( Commands.PING );
	System.out.println( (new Date().getTime()) + " C: waiting for reply from server..." );    

	// Wait for that message to be sent, received and acknowledged
	try {
	    Thread.sleep(5000);                 //take 5
	} 
	catch( InterruptedException ex ) {
	    Thread.currentThread().interrupt();
	}

	// loop sending messages to various entities
	while (( thread.getThreadState() != States.CLOSED ) && 
	       ( thread.getConnectionState() == States.ALIVE )) {
	    
	    // Now send a full set of messages to the nominated
	    // destination, waiting for each to be delivered.
	    sendTestMessages();
	    try {
		Thread.sleep(5000);                 //take 5
	    } 
	    catch(InterruptedException ex) {
		Thread.currentThread().interrupt();
	    }
	    
	} 
	
	// did loop exit because user quit or socket connection was lost?
	if (( thread.getThreadState() == States.CLOSED ) || 
	    ( thread.getConnectionState() != States.ALIVE )) {
	    System.out.println( (new Date().getTime()) + " C: lost connection to server." );
	}

	// make sure everything is cleaned up okay
	System.out.println( (new Date().getTime()) + " C: closing down." );
	thread.close();

    } // end run()


    /**
     * handleCommand()
     *
     * Processes commands sent to the client by another client. This is
     * just a skeleton for now that displays the different kinds of
     * command message received.      
     *
     * This method should be overloaded by a method which handles the
     * specific messages sent to the particular type of client.
     *
     * @param command: a String with the command message to process
     *
     * @return: true, if the command was handled; false otherwise
     *
     */
    public boolean handleCommand( String command, StringTokenizer arguments ) {

	try {
	    if ( command.equals( Commands.SEND )) {
		System.out.println( (new Date().getTime()) + " C: received command: " + command );
		return( true );
	    }
	    else if ( command.equals( Commands.POSE )) {
		System.out.println( (new Date().getTime()) + " C: received command: " + command );
		return( true );
	    }
	    else if ( command.equals( Commands.WHERE )) {
		System.out.println( (new Date().getTime()) + " C: received command: " + command );
		return( true );
	    }
	    else if ( command.equals( Commands.GOTO )) {
		System.out.println( (new Date().getTime()) + " C: received command: " + command );
		return( true );
	    }
	    else if ( command.equals( Commands.MOVE )) {
		System.out.println( (new Date().getTime()) + " C: received command: " + command );
		return( true );
	    }
	    else if ( command.equals( Commands.SNAP )) {
		System.out.println( (new Date().getTime()) + " C: received command: " + command );
		return( true );
	    }
	    else if ( command.equals( Commands.IMAGE )) {
		System.out.println( (new Date().getTime()) + " C: received command: " + command );
		return( true );
	    }
	    else if ( command.equals( Commands.HIDE )) {
		System.out.println( (new Date().getTime()) + " C: received command: " + command) ;
		return( true );
	    }
	    else if ( command.equals( Commands.FOUND )) {
		System.out.println( (new Date().getTime()) + " C: received command: " + command) ;
		return( true );
	    }
	    else if ( command.equals( Commands.SCORE )) {
		System.out.println( (new Date().getTime()) + " C: received command: " + command );
		return( true );
	    }
	    else {
		// unknown command
		System.err.println( (new Date().getTime()) + " C: unknown command received: " + command );
		return( false );
	    }
	}
	catch( NoSuchElementException nsex ) {
	    System.err.println( (new Date().getTime()) + " C: command error: " + nsex.toString() );
	    return( false );
	}
    } // end of handleCommand()
    

    /**
     * setDestination()
     *
     */
    public void setDestination(String destination){
	this.destination = destination;
    }


    /**
     * sendTestMessages()
     *
     * Walk through the suite of messages, finishing with a SHUTDOWN
     * which terminates the run.
     *
     */

    public void sendTestMessages(){
	String cmd = null;
	String content = null;

	switch(counter) {
	     case 1:  cmd = Commands.ACK;    
                      break;

		      // No SETID msg since this is already tested by startup and sending again causes 
		      // the client to be shutdown.
		      //case 2:  cmd = Commands.SETID;    
                      //break;

	     case 2:  cmd = Commands.ERROR;   
		      content = "There was an error!";
                      break;

	     case 3:  cmd = Commands.PING;   
		      content = null;
                      break;

	     case 4:  cmd = Commands.PONG;    
                      break;

	     case 5:  cmd = Commands.SEND;   
		      content = "Whatever";
                      break;

	     case 6:  cmd = Commands.POSE;    
		      content = "7 1 1 3.14";
                      break;

	     case 7:  cmd = Commands.WHERE;
		      content = "7";
                      break;

	     case 8:  cmd = Commands.GOTO;    
		      content = "7 2 2";
                      break;

	     case 9:  cmd = Commands.MOVE;  
		      content = "7 F 5";
                      break;

	     case 10:  cmd = Commands.SNAP;    
		      content = "7";
                      break;

	     case 11: cmd = Commands.IMAGE;    
		      content = "7 4 4 3.14 this-is-an-image";
                      break;

	     case 12: cmd = Commands.HIDE;
		      content = "50 60 (colour red) (shape box)";
                      break;

	     case 13: cmd = Commands.FOUND;
		      content = "55 66 (colour blue) (shape ball)";
                      break;

	     case 14: cmd = Commands.SCORE;    
		      content = "100 20 20 (colour red) (shape box)";
                      break;
		       
	     case 15: cmd = Commands.SHUTDOWN;    
                      break;

	     default: System.out.println( (new Date().getTime()) + " C: Should never get here!");	  
	}
		    
	System.out.println( (new Date().getTime()) + " C: sending to client [" + destination + "]");		
	if ( content == null ) {
	    System.out.println( (new Date().getTime()) + " C: " + cmd + " " + thread.getID() + " " + destination );
	    thread.sendToServer( cmd + " " + thread.getID() + " " + destination );
	}
	else {
	    System.out.println( (new Date().getTime()) + " C: " + cmd + " " + thread.getID() + " " + destination + " " + content );
	    thread.sendToServer( cmd + " " + thread.getID() + " " + destination + " " + content );
	}

	if ( cmd.equals( Commands.SHUTDOWN )) {
	    thread.close();
	}
	else {
	    System.out.println( (new Date().getTime()) + " C: waiting for reply from server..." );
	}

	counter++;

    } // end of sendTestMessage()


    /**
     * sendRandomMessage()
     *
     * Generates a random message and sends it. Covers both messages
     * to server and messages to another client. Just intended to test
     * the message passing, so no guarantees that we won't get strange
     * behaviours.
     *
     */

    public void sendRandomMessage(){
	int    type = rand.nextInt(15) + 1;
	String cmd = null;
	String content = null;

	switch(type) {
	     case 1:  cmd = Commands.ACK;    
                      break;

		      // No SETID msg since this is already tested by startup and sending again causes 
		      // the client to be shutdown.
		      //case 2:  cmd = Commands.SETID;    
                      //break;

	     case 2:  cmd = Commands.ERROR;    
                      break;

	     case 3:  cmd = Commands.PING;    
                      break;

	     case 4:  cmd = Commands.PONG;    
                      break;

	     case 5:  cmd = Commands.SEND;    
                      break;

	     case 6:  cmd = Commands.POSE;    
                      break;

	     case 7:  cmd = Commands.WHERE;    
                      break;

	     case 8:  cmd = Commands.GOTO;    
                      break;

	     case 9:  cmd = Commands.MOVE;    
                      break;

	     case 10:  cmd = Commands.SNAP;    
                      break;

	     case 11: cmd = Commands.IMAGE;    
                      break;

	     case 12: cmd = Commands.HIDE;    
                      break;

	     case 13: cmd = Commands.FOUND;    
                      break;

	     case 14: cmd = Commands.SCORE;    
                      break;

		      // No SHUTDOWN since it shuts down the server and stops the test.
		      //case 15: cmd = Commands.SHUTDOWN;    
                      //break;

	     default: System.out.println( (new Date().getTime()) + " C: Guess I calibrated the random number generator wrong!");	  
	}
		    
	System.out.println( (new Date().getTime()) + " C: sending to client [" + destination + "]");		
	System.out.println( (new Date().getTime()) + " C: " + cmd + " " + thread.getID() + " " + destination + " " + content);
	thread.sendToServer( cmd + " " + thread.getID() + " " + destination + " " + content);
	System.out.println( (new Date().getTime()) + " C: waiting for reply from server..." );

    
    }

} // end of AutoClient class
