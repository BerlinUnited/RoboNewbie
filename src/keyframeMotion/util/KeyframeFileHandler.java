/*******************************************************************************
*  RoboNewbie
* NaoTeam Humboldt
* @author Monika Domanska
* @version 1.1
*******************************************************************************/


package keyframeMotion.util;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import util.RobotConsts;

/*
 * Reads keyframe files from "[Root]/keyframes", where [Root]
 * is the directory where the program runs.
 * @author nika
 */
public class KeyframeFileHandler {
  
  /*! \brief Reads the keyframe file line by line into a KeyframeSequence object.
   * 
   * Skips empty lines and comment lines (starting with "//").
   * 
   * The ordering of angles in a keyframe as defined in RobotConsts (same
   * as in SimSpark-Wiki) must be regarded.
   * 
   */
  public static KeyframeSequence getSequenceFromFile(String fileName) {
    
    KeyframeSequence ks = new KeyframeSequence();
    
	try {
		BufferedReader in = new BufferedReader(new FileReader("keyframes/" + fileName));
		String line = null;
		while ((line = in.readLine()) != null) {
          if (line.length() > 0 && !line.startsWith("//")){
            Keyframe frame = getFrame(line);
            ks.addFrame(frame);
          }
		}
	} catch (Exception e) {
		e.printStackTrace();
	}
    return ks;
  }
  
  /*! \brief Transfers a line from Keyframe.txt into a array of angles. 
   * 
   * It is supposed that each line consists of exactly 22 angles separated by
   * one ore more spaces. Other formats (e.g. tabs or other mistakes) 
   * result in undefined behavior of getFrame. 
   */
  private static Keyframe getFrame(String line){
    int transitionTime;
    double[] angleDoubles = new double[RobotConsts.JointsCount];
    int jointNum = 0;

    String[] valueStrings = line.split(" ");
    
    transitionTime = Integer.parseInt(valueStrings[0]);
    
    for (int i = 1; i < valueStrings.length; i++){
      if (valueStrings[i].length() > 0){
        if (jointNum < angleDoubles.length){
          angleDoubles[jointNum] = (Double.parseDouble(valueStrings[i]));
          jointNum++;
        }
      }  
    }    
    return new Keyframe(transitionTime, angleDoubles);
  }
  
  /*! \brief Stores a sequence into a file. Only for debugging of  NaoTH-MotionEditor.
   * 
   */
  public static void writeSequenceToFile(KeyframeSequence ks, String fileName){
    try{
      FileWriter writer = new FileWriter("keyframes/" + fileName);
      Keyframe kf = ks.getNextFrame();
      while(kf != null)
      {
        StringBuilder sb = new StringBuilder();
        sb.append(kf.getTransitionTime());
        for (int i = 0; i < RobotConsts.JointsCount; i++)
          sb.append(" ").append(kf.getAngle(i));
        sb.append("\n");
        writer.write(sb.toString());
        kf = ks.getNextFrame();
      }
      writer.close();
    }
    catch(Exception e){
      e.printStackTrace();
    }
  }
  
}
