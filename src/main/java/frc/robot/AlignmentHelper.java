package frc.robot;
/**
 *
 * This class represents a sketch of an idea to efficiently line up with the 
 * closest track on the intake.
 * @author Mr. LaSpina
 * 
 */
public class AlignmentHelper {
   int NUMTRACKS = 9; //this is the number of discrete tracks a coral PVC can slide down
   double SPACING = 8; //distance between adjacent track slots.
   //note: The April tag is placed at the middle of the Coral Station.
   
   // positions stores the offset positions for the different tracks of the intake.
   private double[] positions;
   
   //Stores the angles we want to be facing to score each April tag.
   //These numbers assume 0° is facing the driver station and
   //turning left while driving towards the driver station (away from the starting line)
   // is a positive angle turn.
   private int[] angles;
   
   //single object design pattern
   static AlignmentHelper theAligner;
   
   /**
    * Creates the alignment object with discrete posiitons that might
    * get returned by the align function
    */
   private AlignmentHelper(double offset) {
      System.out.println("Building alignment object");
      positions = new double[NUMTRACKS];
      //the zero position is the middle one.
      //compute processor px positions
      //px represents the axis along the processor edge.
      //start with the zero:
      int middleIndex = (NUMTRACKS)/2; //note that middle index is 4 if we have 9 places.
      double px = offset;
      for(int i=middleIndex; i<NUMTRACKS; i++) {
         positions[i] = px;
         px += SPACING;
      }
      //insert values to the left:
      px = offset; 
      for(int i = middleIndex-1; i>=0; i--) {
         px -= SPACING;
         positions[i] = px;
      }
      
      angles = new int[23];
      //index i corresponds to April tag i, so index 0 is not used.
      int angle = 120;
      int tagID;
      for(tagID=6; tagID<=11; tagID++) {
         angles[tagID] = angle;
         angle += 60;
         if(angle==360)
            angle=0;
      }
      angle = -60;
      for(tagID=22; tagID>=17; tagID--) {
         angles[tagID] = angle;
         angle += 60;
      }
      angles[1] = -36;
      angles[2] = 36;
      angles[12] = 36;
      angles[13] = -36;
   }
   
   /**
    * Creates an alignment object used to find the closest discrete position,
    * @param offset represent the px value corresponding to the middle location.
    * If the limelight camera is positioned directly above or below the intake
    * then we would expect this number to be zero. 
    * @return an object
    */
   static public AlignmentHelper getInstance(double offset) {
      if(theAligner==null)
         theAligner = new AlignmentHelper(offset);
         
      return theAligner;
   }
   
   /**
    * 
    * @param tagID The April Tag ID should be from 1 to 22. Only the April
    * Tags on the reef and intakes are used.
    * @return The angle the robot should be rotated to in order to perfectly align
    * with the field element.
    */
   public int getAngleFacingID(int tagID) {
      if(tagID<0 || tagID>22)
         throw new IllegalArgumentException("AlignmentHelper does not recognize"
                 + " an April tag ID of " + tagID);
      return this.angles[tagID];
   }


   /**
    * 
    * @param tagID The April Tag ID should be from 1 to 22. Only the April
    * Tags on the reef and intakes are used.
    * @param offsetAngle Leave this at zero if the intake is mounted to the front
    * of the robot. Use 90° to indicate that the intake is on the left side of the robot.
    * @return 
    */
   public int getAngleFacingID(int tagID, int offsetAngle) {
      if(tagID<0 || tagID>22)
         throw new IllegalArgumentException("AlignmentHelper does not recognize"
                 + " an April tag ID of " + tagID);
      return this.angles[tagID]+offsetAngle;
   }
   /**
    * 
    * TODO: Test this method to verify it works.
    * @param distanceFromCenter is the horizontal offset distance from the center of
    * the coral station. Here a zero indicates that the robot is placed exactly at
    * the correct position to intake from the middle shoot. 
    * @return an integer between 1 and NUMTRACKS. 
    * For example, getClosestShoot(0) should return 5 since the middle shoot
    * is the 5th one counting from the Robot's left
    * getClosestShute(-SPACING) would return 4 since the input indicates
    * that the robot is exactly one SPACING distance to the left of center.
    * getClosestShute(0.5 + SPACING) would return 5 since this 
   */
   public double getClosestShute(double distanceFromCenter) {
     //determine which value in the array is closest to the input.
     double minDistance = 999;
     int bestSpot = 1;
     for(int spot=1; spot<this.NUMTRACKS; spot++) {
        if( Math.abs(positions[spot] - distanceFromCenter) < minDistance) {
           minDistance = Math.abs(positions[spot] - distanceFromCenter);
           bestSpot = spot;
        }
     }
     //return that value so the robot knows where it should move to.
      return bestSpot; 
   }
   
   public static void main(String[] args) {
      AlignmentHelper ia = AlignmentHelper.getInstance(0.5);
      for(double v : ia.positions)
         System.out.print(v + "  ");
      
      //now think about this. What values should these method calls return?
      ia.getClosestShute(50);
      ia.getClosestShute(30);
      ia.getClosestShute(12);
      ia.getClosestShute(-17);
      
      //verified the angles are what we want.
      //for(int i=0; i<ia.angles.length; i++)
      //   System.out.println(i + " : " + ia.angles[i]);       
      
      
      
   }
}
