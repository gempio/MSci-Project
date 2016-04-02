/**
 * ClientThread.java
 *
 * ClientThread includes all objects and methods necessary for talking
 * and listening to the server. All communications to and from the
 * Server occurs via a BufferedReader (input stream) and a PrintWriter
 * (output stream), via a Socket (socket).
 *
 * @author   Elizabeth Sklar
 * @author   Simon Parsons
 * @version  09-jul-1998 (original)
 * @version  01-mar-2015 (revised)
 * @version 17-mar-2015/sklar: added new commands (WHERE, HIDE, FOUND)
 * and removed obselete TREASURE command
 *
 *
 */
import java.io.*;
import java.net.*;
import java.util.*;

public class ClientThread extends Thread {

    String             host;
    int                port;
    String             ID;
    Client             client;  // pointer to the Client that this thread is 
                                // servicing.
    Socket             socket;
    BufferedReader     in;
    PrintWriter        out;
  
    Date               timestamp = new Date();

    static final long  TIMEOUT = 15000;//ms
    long               timeout = TIMEOUT;
  
    // state of the thread
    int                threadState;

    // state of the socket connection to the client
    int                connectionState;


    /** 
     * ClientThread
     *
     * Creates a new generic ClientThread object. This method should
     * be called by the constructor of a class that instantiates
     * objects of this class. It initializies the incoming and
     * outgoing data streams between this class and the server.
     *
     * @param parent: the client that kicked off this communication
     *        thread.
     * @param host: the internet address of the server
     * @param port: the port at which the server is listening for
     *        connections
     * @param ID: an identification string for this client
     *
     */
    public ClientThread( Client parent, String host, int port, String ID ) throws IOException {

	super( "ClientThread" );

	this.client = parent;
	this.host   = host;
	this.port   = port;
	this.ID     = ID;
	
	// create socket
	System.out.println( (new Date().getTime()) + " CT: trying to create socket" );
	socket = new Socket( host,port );
	System.out.println( (new Date().getTime()) + " CT: new socket created a-okay" );

	// open reader on socket
	InputStream inStream = socket.getInputStream();
	in = new BufferedReader( new InputStreamReader( inStream ));

	// open writer on socket
	OutputStream outStream = socket.getOutputStream();
	out = new PrintWriter( new OutputStreamWriter( outStream ));

	// initialize state variables
	setThreadState( States.STARTUP );
	setConnectionState( States.ALIVE );

    } // end of ClientThread constructor



    /**
     * run()
     *
     * Loops continously, waiting for messages from the server and
     * calling handleMessage() to deal with them.
     *
     * @exception EOFException: if the server disappears, print an
     * error message.
     *
     * @exception IOException: if an input or output exception occurs,
     * print a stack trace.
     *
     */
    public void run() {

	String message;
    
	// first send the client's name to the server
	sendToServer( Commands.SETID, getID() );

	// semi-infinite loop, receiving messages from Server through a
	// DataInputStream and sending responses
	setThreadState( States.LISTENING );
	while ( threadState == States.LISTENING ) {
	    try {
		// blocks until there is something to read...
		message = (String)in.readLine();
		if ( message == null ) {
		    break;
		}
		else {
		    timestamp = new Date();
		    if ( connectionState == States.UNCERTAIN ) {
			setConnectionState( States.ALIVE );
		    }
		    System.out.println( (new Date().getTime()) + " CT: received message: " + message );
		    if ( handleMessage( message ) == true ) {
			// gives the successfully handled message a timestamp so
			// that isConnectionAlive can use this to minimize the 
			// amount of pings issued to the client
		    } 
		    else {
			System.out.println( (new Date().getTime()) + " CT: unknown message from server '" + message + "'" );
		    }
		}
	    }
	    catch ( EOFException eofx ) {
		System.err.println( eofx.toString() );
		setThreadState( States.ERROR );
		break;
	    }
	    catch ( IOException iox ) {
		System.err.println( iox.toString() );
		setThreadState( States.ERROR );
	    }
	}
	System.out.println( (new Date().getTime()) + " CT: i'm not listening any more" );
	close();
    } // end of run()


    /**
     * handleMessage()
     *
     * Handles the generic messages sent to the client by the server.
     *
     * @param message: a message from the server. 
     *
     * @return: true, if the message was handled; false otherwise
     *
     */
    public boolean handleMessage( String message ) {

	// tokenize the message
	StringTokenizer arguments = null;
	String          command   = null;

	try {
	    // parse message into command plus arguments
	    arguments = new StringTokenizer( message );
	    command   = arguments.nextToken();
	    // branch based on command
	    if ( command == null ) {
		// nothing to handle
		return( true );
	    }
	    // these are the administrative commands which are handled by the ClientThread.
	    else if ( command.equals( Commands.SETID )) {
		// extract ID from message and set it
		command = arguments.nextToken();
		System.out.println( (new Date().getTime()) + " CT: new client ID is: " + command );
		setID(command);
		return( true );
	    }
	    else if ( command.equals( Commands.ACK )) {
		// acknowledgement received from client
		return( true );
	    }
	    else if ( command.equals( Commands.ERROR )) {
		// error, the message is the third token
		String errorMessage = arguments.nextToken();
		errorMessage = arguments.nextToken();
		errorMessage = arguments.nextToken();
		System.err.println( errorMessage );
		return( true );
	    }
	    else if ( command.equals( Commands.SHUTDOWN )) {
		// shutdown: assume this command comes from the server
		close();
		return( true );
	    }
	    else if ( command.equals( Commands.PONG )) {
		// pong... the response to ping
		// determines that we are still actively connected to the server
		// just needs to return true and let the timestamp take care of 
		// validating the connection in isConnectionAlive
		return( true );
	    }
	    else if ( command.equals( Commands.PING )) {
		// server has sent ping; respond with a pong
		sendToServer( Commands.PONG );
		return( true );
	    }
	    // these are the commands that the client itself need to respond to, so we pass
	    // them back to the client.
	    else if ( command.equals( Commands.SEND )    || 
		      command.equals( Commands.POSE )    || command.equals( Commands.WHERE )  ||
		      command.equals( Commands.GOTO )    || command.equals( Commands.MOVE )  || 
		      command.equals( Commands.SNAP )    || command.equals( Commands.IMAGE ) ||
		      command.equals( Commands.HIDE )    || command.equals( Commands.FOUND ) || 
		      command.equals( Commands.SCORE )) {
		// For now just indicate we got the message.
		//		System.out.println(message);
		return( client.handleCommand( command, arguments ));
	    }
	    else {
		// unknown message
		System.err.println( (new Date().getTime()) + " CT: unknown message received: " + message );
		return( false );
	    }
	}
	catch( NoSuchElementException nsex ) {
	    System.err.println( (new Date().getTime()) + " CT: command error: " + nsex.toString() );
	    return( false );
	}
    } // end of handleMessage()


    /** 
     * sendToServer()
     *
     * Sends a message to a ServerServer, composed of a command
     * followed by a set of arguments.
     *
     * @param command: the command to be sent to the server
     *
     * @param arguments: the arguments required for that command
     *
     */
    public void sendToServer( String command, String arguments ) {
	System.out.println( (new Date().getTime()) + " CT: sending message: " + command + " " + arguments );
	out.println( command + " " + arguments );
	out.flush();
    } // end of sendToServer()
    
    public void sendToServer( String command, StringTokenizer argTokens ) {
	String argString = "";
	try {
	    while ( argTokens.hasMoreTokens() ) {
		argString += argTokens.nextToken();
	    }
	}
	catch( NoSuchElementException nsex ) {
	}
	sendToServer( command, argString );
    } // end of sendToServer()

    public void sendToServer( String command ) {
	sendToServer( command, "" );
    } // end of sendToServer()


    /**
     * setID
     *
     * Sets this ClientThread's ID.
     *
     * @param ID: the name of this ClientThread's ID
     *
     */
    public void setID( String ID ) {
	this.ID = ID;
	//System.out.println( (new Date().getTime()) + " CT: set ID to " + this.ID );
    } // end of setID()


    /**
     * getID
     *
     * Returns this ClientThread's ID
     *
     * @return: the name of this ClientThread's ID
     *
     */
    public String getID() {
	return( ID );
    } // end of getID()


    /**
     * setThreadState()
     *
     * This method sets this ClientThread's state.
     *
     */
    public void setThreadState( int threadState ) {
	this.threadState = threadState;
    } // end of setThreadState()


    /**
     * getThreadState()
     *
     * Returns this ClientThread's state
     *
     * @return: the value of this ClientThread's state
     *
     */
    public int getThreadState() {
	return( threadState );
    } // end of getThreadState()


    /**
     * setConnectionState()
     *
     * This method sets this ClientThread's connectionState.
     *
     */
    public void setConnectionState( int connectionState ) {
	this.connectionState = connectionState;
    } // end of setConnectionState()


    /**
     * getConnectionState()
     *
     * Returns this ClientThread's connectionState.
     *
     * @return ClientThread's connectionState
     *
     */
    public int getConnectionState() {
	return( this.connectionState );
    } // end of getConnectionState()


    /**
     * isConnectionAlive()
     *
     * Checks to see if the connection to the server is still alive.
     * This means that the thread is active AND the ServerClient is
     * still responsive.
     *
     */
    public boolean isConnectionAlive() {
	
	// is the thread alive?
	if ( ! isAlive() ) {
	    return( false );
	}
	
	// have we already concluded that the connection is dead?
	if ( connectionState == States.DEAD ) {
	    System.out.println( (new Date().getTime()) + " CT: client[" + ID + "] connection to server is dead" );
	    return( false );
	}
    
	// if the timestamp has occured before the timeout then a ping is
	// not necessary since we know that the server is still there. 
	// otherwise, we are not certain that the server is alive so a 
	// ping is necessary to provoke a pong from the server.
	Date now = new Date();
	if (( now.getTime() - timestamp.getTime() ) < TIMEOUT ){
	    setConnectionState( States.ALIVE );
	    return( true );
	}
    
	// if connectionState is still uncertain, then no pong was
	// received from the server
	if ( connectionState == States.UNCERTAIN ) {
	    System.out.println( (new Date().getTime()) + " CT: client[" + ID + "] connection to server is uncertain" );
	    setConnectionState( States.DEAD );
	    return( false );
	}
      
	// pings the server; should get a pong back which handleMessage
	// will receive and set connectionState to States.ALIVE.
	sendToServer( Commands.PING );
	setConnectionState( States.UNCERTAIN );

	// assumes the connection is alive for now. next time
	// isConnectionAlive() is called an accurate decision can be made,
	// but first handleMessage must wait for a pong (actually, any 
	// message will do) from the server
	return( true );

    } // end of isConnectionAlive()


    /**
     * close()
     *
     * Frees all communications resources used by this class.  This
     * method is called by finalize().
     *
     */
    public void close() {
	if ( getThreadState() != States.CLOSED ) {
	    try {
		// don't close things that may never have been constructed
		if ( out != null ) {
		    out.close();
		    out = null;
		}
		if ( in != null ) {
		    in.close();
		    in = null;
		}
		if ( socket != null ) {
		    // close the socket and all associated streams
		    socket.close();
		    socket = null;
		}
		setThreadState( States.CLOSED );
		setConnectionState( States.DEAD );
	    }
	    catch ( Exception x ) {
		System.err.println( x.toString() );
	    }
	}
    } // end of close()

} // end of ClientThread class
