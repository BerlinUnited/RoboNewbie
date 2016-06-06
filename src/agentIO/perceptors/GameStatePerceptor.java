/*******************************************************************************
*  RoboNewbie
* NaoTeam Humboldt
* @author Monika Domanska
* @version 1.1
* extended by hdb March 2014
*******************************************************************************/

package agentIO.perceptors;

import static util.GameStateConsts.PlayMode;

/**
 * This class represents the data received by the game state perceptor. 
 * 
 * As all Perceptor-classes this class is just for reading the perceptor values
 * provided by the server.
 * Instances of this class are immutable.
 * The game state perceptor informs about the play time and the play mode.
 * 
 * The first message after initialization (only this message) contains 
 * teamside (= left or right) and unum (= player number) which is needed if
 * the server assigns the numbers to the players.
 * ServerCommunication takes care of these informations.
 * 
 */
public class GameStatePerceptor {
  
  double playTime;
  PlayMode playMode;
  String teamSide;     
  String unum;           


  /**
   * Constructor.
   * 
   * @param playTime The time elapsed since the kick off (since the server has been
   * set to the playmode "KickOff_Left" for the first time) until the game state
   * message has been received. This time value stops during the halftime break. 
   * @param timeSide The side (left or right) of the team as set by the server.
   * @param unum The player number. It is set by in initialization in 
   * the init-message or provided by the server.
   */
  public GameStatePerceptor(double playTime, PlayMode playMode,String teamSide,
			String unum) {
    this.playTime = playTime;
    this.playMode = playMode;
    this.teamSide = teamSide;   //hdb
    this.unum = unum;           //hdb
  }

  /**
   * Returns the play mode. 
   * 
   * @return The actual play mode. 
   */
  public PlayMode getPlayMode() {
    return playMode;
  }

  /**
   * Returns the play time in seconds.
   * 
   * @return The time elapsed since the kick off (since the server has been set 
   * to the playmode "KickOff_Left" for the first time). This time value stops
   * during the halftime break. 
   */
  public double getPlayTime() {
    return playTime;
  }
 
   /**
   * Returns the team side.
   * 
   * @return the side (left or right) of the team.
   * According to that, the team has to score at the right or left goal.
   * It plays to the positive or negative x-direction.
   */
  public String getTeamSide(){
    return teamSide;
  }

   /**
   * Returns the player number.
   * 
   * @return the the player number as initialized or provided by the server.
   */
  public String getAgentNumber(){
   return unum;
  }
  
  

  /**
   * Returns a textual representation of all information stored in class 
   * variables. 
   * 
   * @return Textual representation of this object.
   */
  @Override
  public String toString() {
    return "game state: play time " + playTime + "s, playmode " + playMode
            + "team side " + teamSide + "agent number " + unum;
  }  
}
