package frc.robot;

//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;

public class LimeLightCamera {

  private boolean m_LimelightHasValidTarget = false;

  //  Utility class for timing
  private Delay delay;

      //  Limelight measured parameters
  private double tv;  //  Valid target
  private double tx;  //  X-plane angle
  private double ty;  //  Y-plane angle
  private double ta;  //  Target area as a percentage of image

  double y_height;    //  Vertical distance from target to camera
  double x_distance;  //  Horizontal distance from target to camera
  double bandwidth;

  int count;

  //  Constructor
  LimeLightCamera()
  {
      delay=new Delay();
      m_LimelightHasValidTarget = false;

      //  Set limelight for vision processing
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);

  }

    //  Private member access functions
    public double get_tv()
    {
        return(tv);
    }

    public double get_tx()
    {
        return(ta);
    }

    public double get_ty()
    {
        return(ta);
    }
    public double get_ta()
    {
        return(ta);
    } 

  public int Update_Limelight_Tracking()
  {
      
    //  Read the LimeLight values
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    
    //  might need a small delay to facilitate screen prints?
    delay.delay_milliseconds(15);

    if(count==20)  {

      //  Output to console
      System.out.printf("tv = %.3f\t",tv);
      System.out.printf("tx = %.3f\t",tx);
      System.out.printf("ty = %.3f\t",ty);
      System.out.printf("ta = %.3f\n\n",ta);
      System.out.println("Valid Target? " + m_LimelightHasValidTarget);

      //  Output to smart dashboard
      SmartDashboard.putNumber("LimelightX", tx);
      SmartDashboard.putNumber("LimelightY", ty);
      SmartDashboard.putNumber("LimelightArea", ta);
      SmartDashboard.putNumber("Horizontal Distance",x_distance);
      
      count=0; //  zero output counter
    }

    count++;  //  increment output counter


    //  Set valid target boolean depending on the value of tv
    if (tv < 1.0)
    {
      m_LimelightHasValidTarget = false;
    }  else  {
      m_LimelightHasValidTarget = true;
    }
   
    return(0);
  
  }


  ///////////////////////////////////////////////////////////////////
  //  Function: double get_x_distance( ... )
  ///////////////////////////////////////////////////////////////////
  //
  //  Purpose:  Given the angle of the camera mount from the
  //            horizontal and the y distance from the camera
  //            to the target.  This function computes the
  //            horizontal distance to the target.
  //
  //  Arguments: double theta_camera - the angle in degrees
  //             of the camera mount from the horizontal.
  //             double y - the vertical distance from the
  //             camera lens to the target. 
  //             (Target height - camera height)
  //
  //  Returns:   The x distance to the target from the
  //             camera lens.
  //
  //  Remarks:   Important to keep the units consistent.
  //
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  public double get_x_distance(double theta_camera,double y)
  {
    double theta;
    double x;

    //  total angle in degrees from horizontal to target.
    theta=ty+theta_camera;

    theta*=Math.PI/180.0;  //  Convert to radians

    x=y/Math.tan(theta);

    return(x);
  }


    
}
