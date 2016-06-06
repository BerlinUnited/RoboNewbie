/*******************************************************************************
*  RoboNewbie
* NaoTeam Humboldt
* @author Hans-Dieter Burkhard, 16.10.2014
* @version 1.1
* 
* Program is a modification of Agent_SimpleSoccer written by Monika Domanska
*******************************************************************************/

package examples.agentSoccerTeam;

import agentIO.EffectorOutput;
import agentIO.PerceptorInput;
import agentIO.ServerCommunication;
import agentIO.perceptors.GameStatePerceptor;
import directMotion.LookAroundMotion;
import java.io.IOException;
import java.util.logging.Level;
import keyframeMotion.KeyframeMotion;
import localFieldView.LocalFieldView;
import util.Logger;
import util.GameStateConsts.PlayMode;

/**
 * This agent can be used to start all players of a team.
 * If the player id is set to "0", each call instantiates a new player. 
 * The player numbers are then assigned by the server in the order of starting 
 * the program. 
 * Depending on this number, the players can have different roles to be 
 * specified in class SoccerTeamThinking.
 * 
 * All parameters determining the identity (teamSide, teamName, finalRobotID,
 * finalBeamCoordsX, finalBeamCoordsY, finalBeamCoordsRot) are contained in
 * ServerCommunication.
 * 
 * Usage: 
 * Specify final String team = <name of team> .
 * Run Agent_SoccerTeam for each player you want to have, numbers are 
 * consecutively assigned by the server. 
 * The initial positions (beam-poses) are defined in class util.BeamPoses .
 * The roles of the players are distributed according to their numbers in 
 * class SoccerTeamThinking.
 * You can use the program for the second team as well: 
 * Simply change the team name and proceed as before. 

 * 
 */
public class Agent_SoccerTeam {

  public static void main(String args[]) {
    
    Agent_SoccerTeam agent = new Agent_SoccerTeam();
    
    agent.init();
    
    agent.run();
    
    agent.printlog();
    
    System.out.println("Agent stopped.");
  }

  private Logger log;
  private PerceptorInput percIn;
  private EffectorOutput effOut;
  private KeyframeMotion kfMotion;
  private LocalFieldView localView;
  private SoccerTeamThinking soccerTeamThinking;
  private LookAroundMotion lookAround;
  
  /** A player is identified in the server by its player ID and its team name. 
   There are at most two teams an the field, and every agent of a single team 
   must have a unique player ID between 1 and 11. 
   If the identification works right, it is visualized in the monitor: 
   the robots on the field have either red or blue parts. An unidentified 
   robot has grey parts. 
   With player id set to "0", the program can be used to start all 
   players of a team one after the other as described above.  
   You can use a number between 1 and 11 instead to start a player with that 
   identity.
   */
  final String id = "0";
  final String team = "SoccerTeam";
  
  /** If the player id is set to "0",the "beam"-coordinates given below are
   overwritten by the server according to the definitions in the 
   class util.BeamPoses.
   The "beam"-coordinates specify the robots initial position on the field.
   The root of the global field coordinate system is in the middle of the 
   field, the system is right-handed. The x-axis points to the opponent goal, 
   so the initial position has a negative x-value to beam the robot on its own
   half. The robot can be placed with an initial orientation given in the 
   variable beamRot, in degrees, counterclockwise relative to the x-axis. */
  final double beamX =    -0.3;
  final double beamY =     0.0;
  final double beamRot =   0;
  
  ServerCommunication sc;
  
    PlayMode pm;

  /**
   * Initialize the connection to the server, the internal used classes and 
   * their relations to each other, and create the robot at a specified position 
   * on the field. 
   */
  private void init() {
  
    log = new Logger();

    sc = new ServerCommunication();
    sc.initRobot(id, team, beamX, beamY, beamRot);
   
    String agentNumber = sc.finalRobotID;
    percIn = new PerceptorInput(sc);
    effOut = new EffectorOutput(sc);
    kfMotion = new KeyframeMotion(effOut, percIn, log);
    localView = new LocalFieldView(percIn, log, team, agentNumber);
    lookAround = new LookAroundMotion(percIn, effOut, log);  
    soccerTeamThinking = new SoccerTeamThinking(percIn, localView, kfMotion, log, sc);   
  }
  

  /**
   * Main loop of the agent program, where it is synchronized with the 
   * simulation server. 
   * 
   * How long the agent program will run can be changed in variable 
   * "agentRunTimeInSeconds". This is just an approximation, because this value 
   * is used to calculate a number of server cycles, and the agent will 
   * participate in this amount of cycles. 
   */
  public void run(){
    
    int agentRunTimeInSeconds = 1200;
    
    // The server cycle represents 20ms, so the agent has to execute 50 cycles 
    // to run 1s. 
    int totalServerCycles = agentRunTimeInSeconds * 50;
    
    // This loop synchronizes the agent with the server.
    for (int i = 0; i < totalServerCycles; i++) {
      
      //check for aborting the agent from the console (by hitting return)
      try {
        if (System.in.available() != 0)
          break;
      } catch (IOException ex) {
        java.util.logging.Logger.getLogger(Agent_SoccerTeam.class.getName()).log(Level.SEVERE, null, ex);
      }
      
      sense();     
      
      think();
      
      act();
    }
  }
  
  /**
   * Update the world and robot hardware informations, that means process 
   * perceptor values provided by the server.
   */
  private void sense() {
    // Receive the server message and parse it to get the perceptor values. 
    percIn.update();
    // Get gamestate
       GameStatePerceptor g = percIn.getGameState();
       pm = g.getPlayMode();
    // Proceed and store values of the vision perceptor.
    localView.update();
  }
  
  /**
   * Decide, what is sensible in the actual situation. 
   * Use the knowledge about the field situation updated in sense(), and choose
   * the next movement - it will be realized in act().
   * 
   * For the moment, thinking is restricted to PlayMode PlayOn, but could be changed. 
   * After goals, the robot is beamed to its initial position.
   * Robot performs only lookaround in other playmodes.
   */
  private void think(){
       if (pm == PlayMode.PlayOn){soccerTeamThinking.decide();}
  }
  
  /**
   * Move the robot hardware, that means send effector commands to the server. 
   */
  private void act(){
    /**
     * Calculate effector commands and send them to the server, this method
     * of class KeyframeMotion has to be called in every server cycle. 
     * 
     * If there was a goal, the agent is beamed to his initial position instead.    
   */
     
    if (pm == PlayMode.Goal_Left || pm == PlayMode.Goal_Right)
    { sc.sendBeamMessage();
    }
    else
    {
    kfMotion.executeKeyframeSequence();
    lookAround.look(); // No matter, which move the robot executes, it should
                       // always turn its head around. So the LookAroundMotion
                       // is called after the KeyframeMotion, to overwrite the 
                       // commands for the head. 
    // Send agent message with effector commands to the server.
    effOut.sendAgentMessage();
    }
  }

  /**
   * Print log informations - if there where any. 
   */
  private void printlog() {
    log.printLog();
  }
  
}
