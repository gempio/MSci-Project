/**
 * Client.java
 *
 * Client is an example Java application with a main() method to test
 * Client/Server communication. 
 *
 * The easy way to write a client that is compliant with the Server is
 * to write a subclass that overloads the handleCommand() method.
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

public class Client {

    String       host;
    int          port;
    String       ID;
    ClientThread thread;


    /**
     * Client constructor
     *
     */
    public Client( String host, int port, String ID ) throws IOException {
	
	// instantiate new ClientThread object
	thread = new ClientThread( this, host, port, ID );

	// if we get here, then the socket connection was opened successfully
	thread.start();

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

	Client client;

	try {
	    // get the hostname and port from the command line. 
	    // fatal error if these are not provided. 
	    String host = args[0];
	    int    port = Integer.parseInt( args[1] );
	    String ID   = args[2];

	    // instantiate new Client object
	    client = new Client( host, port, ID );

	    // start up main run thread
	    client.run();

	}
	catch ( IOException iox ) {
	    System.err.println( iox );
	    System.exit( 1 );
	}
	catch( ArrayIndexOutOfBoundsException aioobe ){
	    System.err.println( "usage: java Client <host> <port> <ID>" );
	    System.exit( 1 );
	}	

    } // end of main()


    /**
     * run()
     *
     */
    public void run() {

	// open stream to read user's input
	BufferedReader stdin = new BufferedReader( new InputStreamReader( System.in ));
	String message = "";
	
	// loop while user wants to send messages
	boolean more = true;
	while (( more ) && 
	       ( thread.getThreadState() != States.CLOSED ) && 
	       ( thread.getConnectionState() == States.ALIVE )) {
	    System.out.println( "enter message to send (q to quit): " );
	    try{
		message = (String)stdin.readLine();
		if ( message != null ) {
			if (( message.charAt( 0 ) == 'q' ) && ( message.length() == 1 )) {
			    more = false;
			}
			else {
			    thread.sendToServer( message );
			    System.out.println( (new Date().getTime()) + " C: waiting for reply from server..." );
			}
		    }
		}
	    catch( IOException iox ) {
		System.err.println( iox.toString() );
		more = false;
	    }
	    // end up here if the server quits without a shutdown
	    catch( StringIndexOutOfBoundsException siobex ){
		; // just close gracefully
	    }	    
	} // end while more messages from user

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
    

} // end of Client class
