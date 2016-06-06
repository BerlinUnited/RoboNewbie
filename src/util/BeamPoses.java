/*******************************************************************************
*  RoboNewbie
* NaoTeam Humboldt
* @author hdb
* @version 1.1
*******************************************************************************/

package util;

// import java.util.HashMap;

/**
 * This class contains the poses for beam messages. The values below are only 
 * examples. You can specify according to your needs. The first coordinate (X-value) 
 * has to be negative because the players must start in their own half.<br>
 *
 * These poses are used in ServerCommunication when the player numbers are
 * assigned by the server.
 */
public class BeamPoses {
    
  private static final double [][] Poses = 
 {
        {-5, 0.0, 0.0}, //player 1, take (-5.0, 0, 0) for a goalie
        {-0.5, 0.0, 0.0}, //player 2
        {-1.0, 2.0, 0.0}, //player 3
        {-1.0, -2.0, 0.0}, //player 4
        {-3.0, 0.0, 0.0}, //player 5
        {-3.0, -1.5, 0.0}, //player 6
        {-3.0, 1.5, 0.0}, //player 7
        {-4.0, -2.0, 0.0}, //player 8
        {-4.0, 0.0, 0.0}, //player 9
        {-4.0, 2.0, 0.0}, //player 10
        {-5.0, 0.0, 0.0}  //player 11
    };
           
  public static double getX(int i) {
    return Poses[i-1][0];
  }
  
   public static double getY(int i) {
    return Poses[i-1][1];
  }
   
 public static double getRot(int i) {
    return Poses[i-1][2];
  }
 

}
