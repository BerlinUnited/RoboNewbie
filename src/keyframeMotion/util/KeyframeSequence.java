/*******************************************************************************
*  RoboNewbie
* NaoTeam Humboldt
* @author Monika Domanska
* @version 1.1
*******************************************************************************/

package keyframeMotion.util;

import java.util.ArrayList;

 /** Represents a sequence of keyframes in the memory.
  *
  * Used to retrieve the frames one by one. Stores which frame was read last.
  *
  * @ See Reader classes for reading a sequence from a file.
  */

public class KeyframeSequence {
    
    private ArrayList<Keyframe> sequence = new ArrayList();
    private int nextFrameNumber = 0;
    
    /** Returns the next frame.
    *
    * Output is zero after the last frame. Then it starts again with first frame.
    */
    
    public Keyframe getNextFrame(){
        Keyframe nextFrame;
        if (nextFrameNumber == sequence.size()){
            nextFrameNumber = 0;
            nextFrame = null;
        }
        else {
            nextFrame = sequence.get(nextFrameNumber);
            nextFrameNumber++;
        }
        return nextFrame;
    }
    
    /**
     * Append a new frame to the end of the sequence.
     * 
     * @param frame The new frame to add. 
     */
    public void addFrame(Keyframe frame){
      if(frame != null)
        sequence.add(frame);
    }
    
    public KeyframeSequence(){        
    }    
}
