/**
 * ServerThread.java
 *
 * ServerThread is the class for all server-side communication with
 * Clients. There is one instantiation of this class in the Server
 * for each online Client. Communication to and from with the Client
 * occurs via a BufferedReader (input stream) and a PrintWriter
 * (output stream), via a Socket (socket).
 *
 * @author  Elizabeth Sklar
 * @author  Simon Parsons
 * @version 9-jul-1998 (original)
 * @version 28-feb-2015 (revised)
 * @version 17-mar-2015/sklar: added new commands (WHERE, HIDE, FOUND)
 * and removed obselete TREASURE command
 *
 */
import java.io.*;
import java.net.*;
import java.util.*;

public class ServerThread extends Thread {

    // data structures for enabling socket-based communication with a client
    Server            server          = null;
    Socket            socket          = null;
    BufferedReader    in              = null;
    PrintWriter       out             = null;

    // my client's name, as set by the client
    String            ID              = null;

    // set to the time something was received from the client
    Date              timestamp       = new Date();

    // used to check that connection to client is still ok
    static final long TIMEOUT         = 15000;//ms
    long              timeout         = TIMEOUT;
  
    // state of the thread
    int               threadState     = States.ERROR;

    // state of the socket connection to the client applet
    int               connectionState = States.DEAD;

    // flag to control exiting from the thread
    // 
    // http://docs.oracle.com/javase/1.5.0/docs/guide/misc/threadPrimitiveDeprecation.html
    //
    // suggest this approach to forcing threads to quit rather than
    // the deprecated Thread.stop().
    boolean            keepRunning    = true;

    /** 
     * ServerThread constructor
     *
     * This method creates a new ServerThread object.
     *
     * This method initializes the incoming and outgoing data streams
     * between this class and the ServerThread.
     *
     * @param server: an instance of a subclass of Server
     *
     * @param socket: the socket through which this thread communicates
     * with the Client
     *
     */
    public ServerThread( Server server, Socket socket ) {
	this.server = server;
	this.socket = socket;
	try {
	    // open input stream
	    InputStream inStream  = socket.getInputStream();
	    in = new BufferedReader( new InputStreamReader( inStream ));
	    // open output stream
	    OutputStream outStream = socket.getOutputStream();
	    out = new PrintWriter( new OutputStreamWriter( outStream ));
	}
	catch ( IOException iox ) {
	    System.err.println( (new Date().getTime()) + " ST: client[]: IOException: " + iox );
	    setThreadState( States.ERROR );
	    setConnectionState( States.DEAD );
	    return;
	}
	setThreadState( States.STARTUP );
	setConnectionState( States.ALIVE );
	this.start();
    } // end of ServerThread constructor

    /**
     * run()
     *
     * Loops continously, waiting for messages from the Clients and
     * calling the ServerThread's handleMessage to process them.
     *
     * @exception IOException: if an input or output exception occurs,
     * print a stack trace
     *
     */
    public void run() {
	setThreadState( States.LISTENING );
	while ( keepRunning && getThreadState() == States.LISTENING ) {
	    try {
		// wait for messages from client
		String message = (String)in.readLine();
		if ( message == null ) {
		    // close upon end of input stream
		    System.out.print( (new Date().getTime()) + " ST: received NULL msg from  client[" + ID + "]" );
		    System.out.println( "  entering closing state..." );
		    setThreadState( States.CLOSING );
		}
		else {
		    setTimestamp();
		    if ( getConnectionState() == States.UNCERTAIN ) {
			setConnectionState( States.ALIVE );
		    }
		    System.out.println( (new Date().getTime()) + " ST: received from client[" + ID + "]: " + message );
		    if ( ! handleMessage( message )) {
			System.out.println( (new Date().getTime()) + " ST: received from client[" + ID + "]: " + message );
		    }
		    // send acknowledgement
		    sendToClient( Commands.ACK );
		}
	    }
	    catch ( IOException iox ) {
		System.err.println( (new Date().getTime()) + " ST: client[" + ID + "] has IO exception = " + iox.toString() );
		setThreadState( States.CLOSING );
	    }
	} // end while listening on socket
	System.out.println( (new Date().getTime()) + " ST: now close connection to  client[" + ID + "]" );
	close( true );
    } // end of run()
    
    
    /**
     * stop()
     *
     * The other part of replacing the deprecated Thread.stop() call.
     */
    public void stopThread(){
	keepRunning = false;
    }

    /**
     * handleMessage()
     *
     * Handles the messages sent by the Client.
     *
     * @param message: a String with a message from the Client
     *
     * @return true: if the message was handled;
     *         false: otherwise
     *
     */
    public boolean handleMessage( String message ) {
	
	// tokenize the message
	StringTokenizer arguments = null;
	String          command   = null;
	String          from      = null;
	String          to        = null;
	int             IDFromServer;
	try {
	    // parse message into command plus arguments
	    arguments = new StringTokenizer( message );
	    command   = arguments.nextToken();
	    // branch based on command
	    if ( command == null ) {
		// nothing to handle
		return( true );
	    }
	    // Administrative messages which we handle between the
	    // server and the client it is connected to.
	    else if ( command.equals( Commands.ACK )) {
		// acknowledgement received from client
		return( true );
	    }
	    else if ( command.equals( Commands.SETID )) {
		String id;
		try {
		    id  = arguments.nextToken();
		    // First check ID is unique
		    if( server.find( server.clients, id ) != null ){
			// We already have a client with this name, so kill connection
			System.out.println( (new Date().getTime()) + " ST: client[" + id + "] already exists!" );
			close( true );
		    }
		    else{
			setID( id );		    
		    }
		    return( true );
		}
		catch ( NoSuchElementException nsex ) {
		    System.err.println( (new Date().getTime()) + " ST: received command error: " + nsex.toString() );
		    return( false );
		}
	    }
	    else if ( command.equals( Commands.GETID )) {
		// Set client's id value
		//
		// Not curently used, but left here in case we ever need
	        // to start generating IDs again
		IDFromServer = server.getID();
		// If the ID we are sent is 0, then the server can no
		// longer count clients and we have to reject this
		// one.
		if ( IDFromServer == 0 ){
		    System.out.println( (new Date().getTime()) + " ST: no more clients!" );
		    close( true );
		}
		else{
		    sendToClient( Commands.SETID, Integer.toString( IDFromServer ));
		    setID( Integer.toString( IDFromServer ));
		}
		return( true );
	    }
	    else if ( command.equals( Commands.SHUTDOWN )) {
		// client has sent a SHUTDOWN command to the server
		// this indicates that this client is shutting down, so we close the
		// thread for talking to this client.
		System.out.println( (new Date().getTime()) + " ST: received shutdown from client[" + ID + "]" );
		close( false );
		return( true );
	    } 
	    else if ( command.equals( Commands.PING )) {
		// ping... the client is checking to see if the server is
		// still alive
		sendToClient( Commands.PONG, "i am here" );
		return( true );
	    }
	    else if ( command.equals( Commands.PONG )) {
		// pong... the response to ping. determines if the client
		// is still actively connected; just needs to return true
		// and let the timestamp take care of validating the
		// connection in ServerThread.isConnectionAlive
		return( true );
	    }
	    else if ( command.equals( Commands.SEND )) {
		// send: send the message to another client
		// command syntax = SEND <FROM> <TO> <MESSAGE>
		// detokenize the message to get the recipient and content,
		// and then pass the message content on to the recipient;
		// note that the MESSAGE is further parsed into:
		// <FROM> <TO> <CONTENT>
		try{
		    from = arguments.nextToken();
		    to   = arguments.nextToken();
		    String msgCmd  = arguments.nextToken(); // inside the MESSAGE 
		    String msgFrom = arguments.nextToken(); // inside the MESSAGE
		    String msgTo   = arguments.nextToken(); // inside the MESSAGE
		    String msgContent = "";
		    while ( arguments.hasMoreTokens() ) {
			msgContent += arguments.nextToken() + " ";
		    }
		    sendToID( msgCmd, msgFrom, msgTo, msgContent );
		}
		catch( NoSuchElementException nsex ) {
		    System.err.println( (new Date().getTime()) + " ST: command error: " + nsex.toString() );
		    return( false );
		}
	    }
	    // messages that need to be passed to clients connected to other threads.
	    // for now we just pass these messages along.
	    else if ( command.equals( Commands.ERROR )   || 
		      command.equals( Commands.POSE )    || command.equals( Commands.WHERE ) ||
                      command.equals( Commands.GOTO )    || command.equals( Commands.MOVE )  || 
		      command.equals( Commands.SNAP )    || command.equals( Commands.IMAGE ) ||
		      command.equals( Commands.HIDE )    || command.equals( Commands.FOUND ) || 
		      command.equals( Commands.SCORE )) {
		// Detokenize the message to get the recipient
		try{
		    from = arguments.nextToken();
		    to   = arguments.nextToken();
		    // if we get here, arguments holds the message content
		}
		catch( NoSuchElementException nsex ) {
		    System.err.println( (new Date().getTime()) + " ST: command error: " + nsex.toString() );
		    return( false );
		}
		// if the "to" is my client, then this message came not from my client but from the 
		// ServerThread of the "from" client, so check which client to send it to.
		if( ID == to ){
		    // this is for my client
		    sendToClient( command, from + " " + to + " " + arguments );
		}
		else{
		    // this is for another client, pass the message to the recipient and send ack to 
		    // my client who presumably sent it
		    sendToID( command, from, to, arguments );
		    //sendToClient( Commands.ACK );
		}
		return( true );
	    }
	    // unknown message
	    else {
		System.err.println( (new Date().getTime()) + " ST: unknown message received from client[" + ID + "]: " + message );
		return( false );
	    }
	}
	catch( NoSuchElementException nsex ) {
	    System.err.println( (new Date().getTime()) + " ST: command error: " + nsex.toString() );
	    return( false );
	}
	return( false );
    } // end of handleMessage


    /**
     * setThreadState()
     *
     * This method sets this ServerThread's state.
     *
     */
    public void setThreadState( int threadState ) {
	this.threadState = threadState;
    } // end of setThreadState()


    /**
     * getThreadState()
     *
     * Returns this ServerThread's state.
     *
     * @return ServerThread's state
     */
    public int getThreadState() {
	return( this.threadState );
    } // end of getThreadState()


    /**
     * setConnectionState()
     *
     * This method sets this ServerThread's connectionState.
     *
     */
    public void setConnectionState( int connectionState ) {
	this.connectionState = connectionState;
    } // end of setConnectionState()


    /**
     * getConnectionState()
     *
     * Returns this ServerThread's connectionState.
     *
     * @return ServerThread's connectionState
     *
     */
    public int getConnectionState() {
	return( this.connectionState );
    } // end of getConnectionState()


    /**
     * setID()
     *
     * Sets this ServerThread's ID.
     *
     */
    public void setID( String ID ) {
	System.out.println( (new Date().getTime()) + " ST: setting client name to " + ID );
	this.ID = ID;
    } // end of setID()


    /**
     * getID()
     *
     * Returns this ServerThread's ID.
     *
     * @return the name of this ServerThread's ID.
     */
    public String getID() {
	return( this.ID );
    } // end of getID()


    /**
     * setTimestamp()
     *
     * Sets this ServerThread's timestamp to the current time.
     *
     */
    public void setTimestamp() {
	this.timestamp = new Date();
    } // end of setTimestamp()


    /** 
     * sendToClient()
     *
     * Sends a message to a <code>Client</code>, composed of a command
     * followed by set of arguments. 
     *
     * @param command     the command to be sent, such as ABORT, or
     *                    some game-specific command
     *
     * @param arguments   the remainder of the message, the sender, the
     *                    recipient and the message content.
     */
    public void sendToClient( String command, String arguments ) {
        if ( getConnectionState() == States.DEAD ) {
	    return;
	}
	System.out.println( (new Date().getTime()) + " ST: client[" + ID + "]: sending message " + command + " " + arguments );
	out.println( command + " " + arguments );
	out.flush();
    } // end of sendToClient()

    /**
     * Version to handle a command followed by a set of tokens.
     */
    public void sendToClient( String command, StringTokenizer argTokens ) {
	String argString = "";
	try {
	    while ( argTokens.hasMoreTokens() ) {
		argString += argTokens.nextToken() + " ";
	    }
	}
	catch( NoSuchElementException nsex ) {
	}
	sendToClient( command,argString );
    } // end of sendToClient()

    /**
     * Version to handle just a string
     */  
    public void sendToClient( String command ) {
	sendToClient( command, "" );
    } // end of sendToClient()


    /**
     * sendToID()
     *
     * Sends message to the client with id "ID".
     *
     * command format:
     *   <message type> <from> <to> <content>
     *
     */
    public void sendToID( String messageType, String from, String ID, StringTokenizer content ) {
	ServerThread toClient;
	if (( toClient = server.find( server.clients, ID )) == null ) {
            System.out.println( (new Date().getTime()) + " ST: client[" + ID + "] not found to send message to" );
	    return;
	}
	String contentString = "";
	try {
	    while ( content.hasMoreTokens() ) {
		contentString += content.nextToken() + " ";
	    }
	}
	catch( NoSuchElementException nsex ) {
	}
	toClient.sendToClient( messageType, from + " " + ID + " " + contentString );
    } // end of sendToID()
    public void sendToID( String messageType, String from, String ID, String contentString ) {
	ServerThread toClient;
	if (( toClient = server.find( server.clients, ID )) == null ) {
            System.out.println( (new Date().getTime()) + " ST: client[" + ID + "] not found to send message to" );
	    return;
	}
	toClient.sendToClient( messageType, from + " " + ID + " " + contentString );
    } // end of sendToID()

    /**
     * isConnectionAlive()
     *
     * Checks to see if the connection to the client is still alive.
     * This means that the thread is active AND the Client is still
     * responsive.
     *
     */
    public boolean isConnectionAlive() {
	
	System.out.println( (new Date().getTime()) + " ST: asking client[" + ID + "] are you alive?" );
	
	// have we already concluded that the connection is dead?
	if ( getConnectionState() == States.DEAD ) {
	    System.out.println( (new Date().getTime()) + " ST: connection to client[" + ID + "] is dead" );
	    return( false );
	}
	
	// if the timestamp has occured before the timeout then a ping is
	// not necessary since we know that the client is still there. 
	// otherwise, we are not certain that the client is alive so a 
	// ping is necessary to provoke a pong from the client.
	Date now = new Date();
	if (( now.getTime() - timestamp.getTime() ) < timeout ){
	    System.out.println( (new Date().getTime()) + " ST: client[" + ID + "] says yes i am healthy and alive");
	    setConnectionState( States.ALIVE );
	    return( true );
	}
    
	// if state is still uncertain, then no pong was received from
	// the client
	if ( getConnectionState() == States.UNCERTAIN ) {
	    System.out.println( (new Date().getTime()) + " ST: killing uncertain connection to client[" + ID + "]");
	    setConnectionState( States.DEAD );
	    return( false );
	}
      
	// pings the client; should get a pong back which handleMessage
	// will receive and set state to ALIVE. if server is closing down
	// this client, then send a shutdown message instead of a ping.
	if ( getThreadState() == States.CLOSING ) {
	    sendToClient( Commands.SHUTDOWN );
	}
	else {
	    sendToClient( Commands.PING );
	    setConnectionState( States.UNCERTAIN );
	}
	
	// assumes the connection is alive for now. next time
	// isConnectionAlive() is called an accurate decision can be made,
	// but first handleMessage must wait for a pong (actually, any 
	// message will do) from the client
	return( true );
	
    } // end of isConnectionAlive()


    /**
     * close()
     *
     * Disposes of resources used by this class. Closes the incoming and
     * outgoing communication streams and this Client's socket.
     *
     * @param shutdown: true if a "shutdown" command should sent to the
     * client prior to closing the socket connection to the client;
     * false otherwise.
     *
     */
    public void close( boolean shutdown ) {

	System.out.println( (new Date().getTime()) + " ST: inside close() for client[" + ID + "]" );

	// make sure we aren't already closed
	if ( getThreadState() == States.CLOSED ) {
	    return;
	}

	System.out.println( (new Date().getTime()) + " ST: closing connection to client[" + ID + "]..." );

	// send a message to client that connection is being shut down,
	// if necessary
	if ( shutdown ) {
	    sendToClient( Commands.SHUTDOWN );
	}
	setThreadState( States.CLOSING );
	setConnectionState( States.DEAD );

	// now shut down the connection
	try {
	    if ( in != null ) {
		in.close();
		in = null;
	    }
	    if ( out != null ) {
		out.close();
		out = null;
	    }
	    if ( socket != null ) {
		socket.close();
		socket = null;
	    }
	}
	catch ( Exception x ) {
	    System.err.println( (new Date().getTime()) + " ST: error closing socket for client[" + ID + "] " + x.toString() );
	}
	setThreadState( States.SHUTDOWN );
	System.out.println( (new Date().getTime()) + " ST: connection to client[" + ID + "]: closed." );

    } // end of close()
    
} // end of ServerThread class
