/**
 * Commands.java 
 * 
 * Commands is a class that defines all the commands used for 
 * Client-Server communications. 
 * 
 * @author  Elizabeth Sklar 
 * @version 04-sep-1998 (original) 
 * @version 17-mar-2015/sklar: added new commands (WHERE, HIDE, FOUND)
 * and removed obselete TREASURE command
 *
 */ 
 
public class Commands { 
     
    //----- 
    // ADMINISTRATIVE MESSAGES 
    //----- 
 
    // sent from server to client indicating that a message was received 
    public static final String ACK         = "%%ack"; 
 
   // sent from client to server to get client's ID when a client starts up 
    public static final String GETID       = "%%getid"; 

    // sent from server to client to set the client ID
    public static final String SETID       = "%%setid"; 
    
    // sent from server to all clients to indicate that server is shutting down 
    // OR 
    // sent from client to server to indicate that client is shutting down 
    public static final String SHUTDOWN    = "%%sshhuuttddoowwnn"; 
     
    // sent from one client to another to indicate that an error has occurred 
    public static final String ERROR       = "%%error";  
     
    // server/client checking on each other's status 
    public static final String PING        = "%%ping"; 
    public static final String PONG        = "%%pong"; 
     
 
    //----- 
    // GENERIC MESSAGE 
    //----- 
 
    // an unspecified message, which can be sent from one client to 
    // another, through the server 
    public static final String SEND        = "%%send"; 
     
 
    //----- 
    // ROBOT MOTION MESSAGES 
    //----- 
 
    // pose specification 
    // reports the location of a robot 
    // command format: POSE <robot> <x> <y> <heading> 
    // where 
    // <robot> is the ID of the robot in the specified location 
    // (<x>,<y>) is the 2D position of the robot within the arena's coordinate system 
    // <heading> is the direction that the robot is facing, in degrees (0o is due East) 
    public static final String POSE        = "%%pose"; 
     
    // pose requestion specification 
    // requests the location of a robot 
    // command format: WHERE <robot>
    // where 
    // <robot> is the ID of the robot whose location is being requested
    public static final String WHERE      = "%%where"; 
     
    // goto specification 
    // indicates that the robot should go to a particular location 
    // command format: GOTO <robot> <x> <y> [ <heading> ] 
    // (heading is optional) 
    // where 
    // <robot> is the ID of the robot that should go to the specified location 
    // (<x>,<y>) is the 2D position of the robot within the arena's coordinate system 
    // <heading> is the direction that the robot is facing, in degrees (0o is due East) 
    public static final String GOTO        = "%%goto"; 
     
    // move specification 
    // indicates that the robot should move in a certain way 
    // command format: MOVE <robot> <direction> <intensity> 
    // where 
    // <robot> is the ID of the robot that should execute the move command 
    // <direction> = { F, B, C, c, A, a } 
    //  F=forward; B=backward;  
    //  C=clockwise, wide turn; c=clockwise turn in place; 
    //  A=anti-clockwise, wide turn; a=anti-clockwise turn in place 
    // <intensity> = (1..5) 
    //  5=most and 1=least 
    public static final String MOVE        = "%%move"; 
     
 
    //----- 
    // ROBOT SENSING MESSAGES 
    //----- 
 
    // command for robot to take a picture 
    // command format: SNAP <robot> 
    // where 
    // <robot> is the ID of the robot that should take the picture 
    // note that this assumes that the robot will take the picture 
    // from its current (x,y) location and heading. if a particular 
    // (x,y,heading) is desired for taking the picture, then SNAP 
    // should be preceded by a GOTO command 
    public static final String SNAP        = "%%snap"; 
 
    // image specification 
    // command format: IMAGE <robot> <x> <y> <h> <image-data> 
    // where 
    // <robot> is the ID of the robot that took the picture 
    // (<x>,<y>) is the location of the robot when the image was taken 
    // <h> is the heading of the robot (in degrees) when the image was taken 
    // <image-data> is the content of the image (SVG format) 
    public static final String IMAGE       = "%%image"; 
     
 
    //----- 
    // TREASURE HUNT GAME MESSAGES 
    //----- 
 
    // hide treasure specification: indicates that a treasure with the
    // particular properties is being hidden at location (x,y)
    // command format: HIDE <x> <y> <properties> 
    // where  
    // <x,y> is the location of the treasure 
    // <properties> is a list of name-value pairs, 
    // e.g., (colour red) 
    public static final String HIDE        = "%%hide"; 
 
    // treasure found specification: indicates that a treasure with
    // the particular properties has been found at location (x,y)
    // command format: FOUND <x> <y> <properties> 
    // where  
    // <x,y> is the location of the treasure 
    // <properties> is a list of name-value pairs, 
    // e.g., (colour red) 
    public static final String FOUND       = "%%found"; 
 
    // score specification 
    // command format: SCORE <points> <x> <y> <properties> 
    // where 
    // <points> is the number of points earned by the human-robot team for a particular treasure 
    // <x,y> is the location of the treasure 
    // <properties> is a list of name-value pairs, 
    // e.g., (colour red) 
    public static final String SCORE       = "%%score"; 
 
 
     
} // end of class Commands 
