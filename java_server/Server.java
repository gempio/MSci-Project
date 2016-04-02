/**
 * Server.java
 *
 * The Server is the main class for the ThreadedServer.
 * The Server watches a specified port for new Clients.
 * When one arrives, the Server instantiates a ServerThread to carry
 * out all further communication with that particular Client.
 * The main Server thread then resumes listening for more Clients.
 *
 * @author  Elizabeth Sklar
 * @author  Simon Parsons
 * @version 09-jul-1998 (original)
 * @version 01-mar-2015 (original)
 *
 */

import java.net.*;
import java.io.*;
import java.util.*;


public class Server extends Thread {
  
    // socket and port for communications
    ServerSocket  serverSocket;
    int           port;

    // state of the server
    int           serverState;

    // cleanup thread
    ServerCleanup julio;

    // vector of clients each of which is a ServerThread
    Vector<ServerThread> clients = new Vector<ServerThread>();
    
    /** 
     * Server constructor
     *
     * Creates a new generic Server object. This method should be
     * called by the constructor of a class that extends Server. It
     * initializes shared Server components and instantiates the
     * ServerCleanup helper class.
     *
     * @param port: the port on which the Server should listen for
     * Clients
     *
     */
    public Server( int port ) {

	super();
	
	setServerState( States.STARTUP );

	// initialize port for communicating to clients
	this.port = port;
    
	// set up to accept a socket connection on port. if port has
	// value 0, then the constructor uses a free port.
	try {
	    serverSocket = new ServerSocket( port );
	} 
	catch ( IOException iox ) {
	    System.err.println( (new Date().getTime()) + " S: could not listen on port: " + port + " " + iox );
	    System.err.println( (new Date().getTime()) + " S: aborting <<<<<<<<<<<<<<<<<<<" );
	    System.exit( 1 );
	}

	// output information on where server is listening
	System.out.println( (new Date().getTime()) + " S: server socket created: " );
	System.out.println( (new Date().getTime()) + " S: port = "+serverSocket.getLocalPort() );
	// System.out.println( (new Date().getTime()) + " S: local host address = "+serverSocket.getInetAddress().getHostAddress() );
	// simplified code from:
	// http://stackoverflow.com/questions/9481865/how-to-get-ip-address-of-current-machine-using-java
	try {
	    boolean found = false;
	    InetAddress inetAddr = null;
	    // Iterate all NICs (network interface cards)...
	    for ( Enumeration ifaces = NetworkInterface.getNetworkInterfaces(); ifaces.hasMoreElements() && !found; ) {
		NetworkInterface iface = (NetworkInterface) ifaces.nextElement();
		// Iterate all IP addresses assigned to each card...
		for ( Enumeration inetAddrs = iface.getInetAddresses(); inetAddrs.hasMoreElements() && !found; ) {
		    inetAddr = (InetAddress)inetAddrs.nextElement();
		    if ( ! inetAddr.isLoopbackAddress() ) {
			// Found non-loopback site-local address. Return it immediately...
			found = true;
			System.out.println( (new Date().getTime()) + " S: host name = " + inetAddr.getLocalHost() );
			System.out.println( (new Date().getTime()) + " S: host address = " + inetAddr.getHostAddress() );
			break;
		    }
		}
	    }
	}
	catch ( Exception e ) {
	    System.out.println( (new Date().getTime()) + " S: exception: " + e.toString() );
	}
    
	// create the Cleanup thread
	julio = new ServerCleanup( this );
    
	// ready, set, go!
	setServerState( States.LISTENING );
	this.start();

    } // end of Server constructor


    /**
     * main()
     *
     * Server is executed as a Java application, and this is the main
     * program for that application.
     *
     * The command line takes one option argument, the port number
     * which the Server will use to communicate with its clients.  If
     * a port number is not provided, then the Server uses the next
     * available port number, as provided by the system.
     *
     */
    public static void main( String[] args ) throws IOException {

	int port;

	// Extract a port number from the command line argument if one
	// exists.
	try{
	    port = Integer.parseInt( args[0] );
	}
	// If no port number provided, use a value of 0. This forces
	// the Socket constructor to find a free port by its own
	// devious methods.
	catch( ArrayIndexOutOfBoundsException aioobe ) {
	    System.out.println( (new Date().getTime()) + " S: no port provided, so finding next available" );
	    port = 0;
	}
	Server server = new Server( port );

    } // end of main()


    /**
     * run()
     *
     * This method loops continuously, waiting for ServerThreads and
     * then creating objects to deal with them.
     * 
     * @exception IOException: if an input or output exception occurred
     *
     */
    public void run() {
	// listen for new clients
	System.out.println( (new Date().getTime()) + " S: listening..." );
	try {
	    while ( getServerState() == States.LISTENING ) {
		// wait until a new client comes to town
		clients.add( new ServerThread( this, serverSocket.accept() ));
	    }
	}
	catch ( IOException iox ) {
	    System.err.println( (new Date().getTime()) + " S: " + String.valueOf( iox ));
	}
	catch ( Exception x ) {
	    System.err.println( (new Date().getTime()) + " S: " + String.valueOf( x ));
	}
	close();
    } // end of run()


    /**
     * setServerState()
     *
     * This method sets the Server's state.
     *
     */
    public void setServerState( int serverState ) {
	this.serverState = serverState;
    } // end of setServerState()


    /**
     * getServerState()
     *
     * Returns the Server's state.
     *
     * @return Server's state
     */
    public int getServerState() {
	return( this.serverState );
    } // end of getServerState()

    
    /**
     * getID()
     *
     * Obtain a new id for a new client provided we can count high enough. We don't
     * currently use this, rather we let each client suggest its own ID.
     *
     * To use it, make id a data element of the class.
     */
    public int getID(){
	int id = 1;

	if(id < Integer.MAX_VALUE){
	    return id++;
	}
	else{
	    return 0;
	}
    }

    /**
     * close()
     *
     * This method closes down the Server thread and exits the
     * application. This method will be called by a client that receives
     * a SHUTDOWN command. Otherwise this method would never be called,
     * since the thread blocks in serverSocket.accept(). So the call in
     * this method to serverSocket.close() may be ineffective and the
     * serverSocket = null may be the only thing that works.
     *
     */
    public final void close() {
	ServerThread c;
	int          i;
	// close...
	setServerState( States.SHUTDOWN );
	System.out.println( (new Date().getTime()) + " S: stopping cleanup" );
	julio.stopThread();
	System.out.println( (new Date().getTime()) + " S: killing server socket" );
	// we should do something clean like this first in order
	// to get the attention of the thread, which is 
	// blocking in serverSocket.accept() :
	// but this doesn't work so we have to force it by
	// just setting it to null
	serverSocket = null;
	System.out.println( (new Date().getTime()) + " S: closing " + clients.size() + " clients" );
	for ( i=0; i<clients.size(); i++ ) {
	    c = (ServerThread)clients.get( i );
	    if ( c != null ) {
		c.close( true );
	    }
	}
	System.out.println( (new Date().getTime()) + " S: exiting >>>>>>>>>>>>>>>>>>>" );
	System.exit( 0 );
    } // end of close()


    /**
     * find()
     *
     * This method returns a ServerThread pointer to the client in the
     * argument clients vector which matches this ID; returns null if ID
     * is not found.
     *
     */
    public ServerThread find( Vector them, String ID ) {
	ServerThread c;
	for ( int i=0; i<them.size(); i++ ) {
	    c = (ServerThread)them.get( i );
	    if ( c.ID == null ) {
		return( null );
	    }
	    else if ( c.ID.equals( ID )) {
		return( c );
	    }
	}
	return( null );
    } // end of find()


    /**                                                                                 
     * who()
     *
     * This method returns a string of ID's of all clients and their
     * current state.
     *
     * Note that we always check that client has an ID. This is done
     * because it can happen that one client invokes this routine
     * while another client is in the process of starting up and has
     * not read its ID yet.
     *                                                                                  
     */
    public String who( Vector them ) {
	String w = new String( "" );
	ServerThread client;
	if ( them == null ) {
	    w += "(none)";
	}
	else {
	    for ( int i=0; i<them.size(); i++ ) {
		client = (ServerThread)them.get( i );
		if ( client.getID() != null ) {
		    w += client.getID() + " (" + States.toString( client.getThreadState() ) + ") ";
		}
	    }
	    w += " |";
	}
	return( w );
    } // end of who()

} // end of Server class
