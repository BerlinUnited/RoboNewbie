/*******************************************************************************
*  RoboNewbie
* NaoTeam Humboldt
* @author Monika Domanska
* @version 1.1
*******************************************************************************/

package agentIO.perceptors;

import java.util.HashMap;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import util.Logger;
import util.RobotConsts.BodyPartName;
import static util.RobotConsts.getBodyPartName;

/**
 * This class represents the raw value for a line from the vision perceptor, 
 * for a convenient access to line coordinates see package localfieldView.
 * 
 */ 

/* An object of class PlayerVisionPerceptor provides access to the data of a
 * robot perceived by the camera: team name, player ID, coordinates of
 * visible body parts. Coordinates are given relative to the observing robot.
 *
 * A PlayerVisionPerceptor object is used only for reading the data provided by the
 * simulation server returns. It is immutable to prevent from changes.
 * 
 * @see util.RobotConsts.BodyPartName for names of bodyparts.
 */
public class PlayerVisionPerceptor {
  
  private String team;
  private String ID;
  private HashMap<BodyPartName, Vector3D> bodyParts;
  
/**
 * This class represents the raw value for a line from the vision perceptor, 
 * for a convenient access to line coordinates see package localfieldView.
 */
  public PlayerVisionPerceptor(String team, String ID, HashMap<String, Vector3D> bodyParts){
    this.team = team;
    this.ID = ID;
    this.bodyParts = new HashMap<>();
    for (String s : bodyParts.keySet()) 
      this.bodyParts.put(getBodyPartName(s), bodyParts.get(s));
  }
 
/**
 * This class represents the raw value for a line from the vision perceptor, 
 * for a convenient access to line coordinates see package localfieldView.
 */
  public String getID(){
    return ID;
  }
  
/**
 * This class represents the raw value for a line from the vision perceptor, 
 * for a convenient access to line coordinates see package localfieldView.
 */
  public String getTeam(){
    return team;
  }
  
/**
 * This class represents the raw value for a line from the vision perceptor, 
 * for a convenient access to line coordinates see package localfieldView.
 */
  public Vector3D getBodyPart( BodyPartName b){
    return bodyParts.get(b);
  }
  
/**
 * This class represents the raw value for a line from the vision perceptor, 
 * for a convenient access to line coordinates see package localfieldView.
 */
  public HashMap<BodyPartName, Vector3D> getAllBodyParts(){
    return bodyParts;
  }
  
/**
 * This class represents the raw value for a line from the vision perceptor, 
 * for a convenient access to line coordinates see package localfieldView.
 */
  @Override
  public String toString(){
    StringBuilder retStr = new StringBuilder();
    retStr.append("Player: ").append(team).append(' ').append(ID).append(' ');
    for (BodyPartName b : BodyPartName.values())
      if (bodyParts.get(b) != null)
        retStr.append(b).append(' ').append(Logger.polarStr(bodyParts.get(b))).append(' ');
    return retStr.toString();
  }
  
  
}
