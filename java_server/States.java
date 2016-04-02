/**
 * States.java
 *
 * States is a class that defines all the state codes used by the
 * Threaded Server and Clients.
 *
 * @author   Elizabeth Sklar
 * @version  04-sep-1998 (original)
 *
 */
public class States {
    
    public static final int NO_STATE  = -1;
    
    // thread process states
    public static final int OKAY      = 0;
    public static final int ERROR     = 1;
    public static final int STARTUP   = 2;
    public static final int LISTENING = 3;
    public static final int CLOSING   = 4;
    public static final int SHUTDOWN  = 5;
    public static final int CLOSED    = 6;
    
    // connection states
    public static final int DEAD      = 10;
    public static final int ALIVE     = 11;
    public static final int UNCERTAIN = 12;
    
    /**
     * toString()
     *
     * This method returns a string indicating the meaning of the
     * state argument.
     *
     */
    public static String toString( int state ) {
	switch( state ) {
	case NO_STATE:
	    return( new String( "no state" )); 
	case OKAY:
	    return( new String( "okay" ));
	case ERROR:
	    return( new String( "error" ));
	case STARTUP:
	    return( new String( "startup" ));
	case LISTENING:
	    return( new String( "listening" ));
	case CLOSING:
	    return( new String( "closing" ));
	case SHUTDOWN:
	    return( new String( "shutdown" ));
	case CLOSED:
	    return( new String( "closed" ));
	case DEAD:
	    return( new String( "dead" ));
	case ALIVE:
	    return( new String( "alive" ));
	case UNCERTAIN:
	    return( new String( "uncertain" ));
	}
	return( String.valueOf( state ));
    } // end of toString()

} // end of class States
