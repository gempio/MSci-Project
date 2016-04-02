/**
 * DumbClient.java
 *
 * A Client that doesn't do anything. Helpful for testing.
 *
 * @author  Elizabeth Sklar
 * @author  Simon Parsons
 * @version 09-jul-1998 (original)
 * @version 28-feb-2015 (revised)
 * @version 17-mar-2015/sklar: added new commands (WHERE, HIDE, FOUND)
 * and removed obselete TREASURE command
 *
 *
 */
import java.io.*;
import java.net.*;
import java.util.*;

public class DumbClient extends Client {

    /**
     * Client constructor
     *
     */
    public DumbClient( String host, int port, String ID ) throws IOException {
	
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

	DumbClient client;

	try {
	    // get the hostname and port from the command line. 
	    // fatal error if these are not provided. 
	    String host = args[0];
	    int    port = Integer.parseInt( args[1] );
	    String ID   = args[2];

	    // instantiate new Client object
	    client = new DumbClient( host, port, ID );

	    // start up main run thread
	    client.run();

	}
	catch ( IOException iox ) {
	    System.err.println( iox );
	    System.exit( 1 );
	}
	catch( ArrayIndexOutOfBoundsException aioobe ){
	    System.err.println( "usage: java DumbClient <host> <port> <myID> <destinationID>" );
	    System.exit( 1 );
	}	

    } // end of main()

    /**
     * run()
     *
     */
    public void run() {

	// loop doing nothing, but responding to incoming messages.
	while (( thread.getThreadState() != States.CLOSED ) && 
	       ( thread.getConnectionState() == States.ALIVE )){
	    
	    try {
		Thread.sleep(5000);                 //take 5
	    } catch(InterruptedException ex) {
		Thread.currentThread().interrupt();
	    }
	} 

	if (( thread.getThreadState() == States.CLOSED ) || 
	    ( thread.getConnectionState() != States.ALIVE )) {
	    System.out.println( (new Date().getTime()) + " C: lost connection to server." );
	}
    
	// if we get here someone killed us, so make sure everything is cleaned up okay
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
    
} // end of DumbClient class
