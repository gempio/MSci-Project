/**
 * ServerCleanup.java
 *
 * ServerCleanup is the class used by the Server to clean up old
 * client threads that have died.
 *
 * @author     Elizabeth Sklar
 * @author     Simon Parsons
 * @version    9-jul-1998 (original)
 * @version    28-feb-2015 (revised)
 *
 */
import java.util.*;

public class ServerCleanup extends Thread {
  
    Server server;

    public static final long CLEANUP_INTERVAL =  30000;//ms
    long   cleanupInterval = CLEANUP_INTERVAL;

    // flag to control exiting from the thread
    // 
    // http://docs.oracle.com/javase/1.5.0/docs/guide/misc/threadPrimitiveDeprecation.html
    //
    // suggest this approach to forcing threads to quit rather than
    // the deprecated Thread.stop().
    boolean            keepRunning    = true;
    

    /**
     * ServerCleanup constructor
     *
     * ServerCleanup is used by the Server to clean up after clients
     * that have exited the system.
     */
    ServerCleanup( Server server ) {
	super( "ServerCleanup" );
	this.server = server;
	this.start();
    } // end of ServerCleanup constructor


    /**
     * run()
     *
     * Runs "periodically", cleaning up resources used by clients that
     * are no longer in the system.
     * 
     */
    public void run() {
	while ( keepRunning ) {
	    try {
		sleep( cleanupInterval );
	    }
	    catch( InterruptedException ix ) {
	    }
	    clean();
	}
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
     * cleanClient()
     *
     * checks one client and, if necessary, cleans it up
     *
     * returns "true" if client is cleaned and should be removed from
     * the system (i.e. the server); false otherwise
     *
     */
    public boolean cleanClient( ServerThread client ) {

	// check if client has been shutdown; this means it is ready
	// to be removed
	if ( client.getThreadState() == States.SHUTDOWN ) {
	    return( true );
	}

	// check if client is in the process of closing; this means it is
	// not yet ready to be removed
	if ( client.getThreadState() == States.CLOSING ) {
	    return( false );
	}

	// check if client has died on its own; this means that it can be
	// removed
	if ( ! client.isConnectionAlive() ) {
	    System.out.println( (new Date().getTime()) + " SC: cleanup: deleting dead client " + client.getID() );
	    //client.close( true );
	    client.stopThread();
	    return( true );
	}
	
	// otherwise, assume client is healthy and alive so just leave it
	// alone to live its own life
	return( false );

    } // end of cleanClient()


    /**
     * clean()
     *
     * cleans up resources used by clients that are no longer in the
     * system.
     * 
     */
    public void clean() {
	ServerThread client;
	int          i;
	Date         now;
	System.out.println( (new Date().getTime()) + " SC: cleaning..." );
	System.out.println( server.who(server.clients) );
	// delete clients that are no longer alive
	try {
	    for ( i=0; i<server.clients.size(); i++ ) {
		client = (ServerThread)server.clients.get( i );
		if ( cleanClient( client ) ) {
		    server.clients.remove( i );
		}
	    }
	}
	catch( ArrayIndexOutOfBoundsException aioobx ) {
	    System.err.println( (new Date().getTime()) + " SC: cleanup: " + aioobx );
	}
	System.out.println( (new Date().getTime()) + " SC: done cleaning." );
    } // end of clean()

} // end of ServerCleanup class
