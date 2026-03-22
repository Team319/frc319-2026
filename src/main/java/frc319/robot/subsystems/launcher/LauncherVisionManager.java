package frc319.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc319.lib.subsystem.KinematicsManager;
import frc319.lib.util.LimelightHelpers;
import frc319.robot.Constants.LimelightConstants;
import frc319.robot.Robot;
import frc319.robot.subsystems.vision.Limelight;

import org.littletonrobotics.junction.Logger;

public class LauncherVisionManager extends SubsystemBase {
    private static LauncherVisionManager instance;

    public LauncherVisionManager(){
        if (instance != null) {
            throw new IllegalStateException("LauncherVisionManager already initialized!");
        }
        instance = this;
    }

    public static LauncherVisionManager getInstance() {
        // Lazy load logic could go here, but for Subsystems, constructor is better
        return instance;
    }

    @Override
    public void periodic() {
        // Update the Limelight's heading based on the turret's current pose
        updateLimelightHeading();

        Logger.recordOutput("LaunchingVisionManager/2dDistanceToCurrentTarget", get2dDistanceToCurrentTarget());
        Logger.recordOutput("LaunchingVisionManager/GlobalPoseFromVision", getGlobalPoseFromVision());

    }

    public boolean isTargetVisible(){
        return Limelight.isValidTargetSeen(LimelightConstants.Device.TURRET);
    }

    public boolean isTurretPoseValid() {
    if (!isTargetVisible()) return false;
    
    // Require at least 2 tags for unambiguous 6DOF solve
    boolean tagVisable = NetworkTableInstance.getDefault()
        .getTable("limelight-turret")
        .getEntry("tv")  
        .getBoolean(false);
    
    return tagVisable ;
}

    public Distance get2dDistanceToCurrentTarget(){
        return get2dDistance(LaunchingSolutionManager.getInstance().getTargetPose().getTranslation());
    }

    public Distance get2dDistance(Translation3d target){
        
        if(!isTargetVisible()){

            return Robot.m_robotContainer.turret.getDistance2d(LaunchingSolutionManager.getInstance().getTargetPose());
        }
        else{

            double [] poseBuf = Limelight.getBotPose(LimelightConstants.Device.TURRET);
            Pose3d visionPose = new Pose3d(
                                new Translation3d(poseBuf[0],poseBuf[1],poseBuf[2]), 
                                new Rotation3d(Units.degreesToRadians(poseBuf[3]), Units.degreesToRadians(poseBuf[4]),Units.degreesToRadians(poseBuf[5]))
                              );

            Translation3d myTrans = visionPose.getTranslation();

            return Meters.of(Math.hypot(target.getX() - myTrans.getX(), target.getY() - myTrans.getY()));
            
        }
    }

    public Pose3d getGlobalPoseFromVision(){
        if(Limelight.isValidTargetSeen(LimelightConstants.Device.TURRET)){
            double [] poseBuf = Limelight.getBotPose(LimelightConstants.Device.TURRET);
        
            // TODO - add protection to make sure poseBuf is big enough...
            Pose3d visionPose = new Pose3d(
                    new Translation3d(poseBuf[0],poseBuf[1],poseBuf[2]), 
                    new Rotation3d(Units.degreesToRadians(poseBuf[3]), Units.degreesToRadians(poseBuf[4]),Units.degreesToRadians(poseBuf[5]))
                    );

            return visionPose;
        }
        else
        {
            return new Pose3d();//KinematicsManager.getInstance().getGlobalPoseFor(Robot.m_robotContainer.turret);
        }
    }

    // In LauncherVisionManager
public Pose3d getTurretCameraPoseInField() {
    double[] buf = NetworkTableInstance.getDefault()
        .getTable("limelight-turret")
        .getEntry("botpose_wpiblue")
        .getDoubleArray(new double[6]);
    return new Pose3d(
        new Translation3d(buf[0], buf[1], buf[2]),
        new Rotation3d(
            Units.degreesToRadians(buf[3]),
            Units.degreesToRadians(buf[4]),
            Units.degreesToRadians(buf[5])
        )
    );
}


    private void updateLimelightHeading(){

        Pose2d currentTurretPose = KinematicsManager.getInstance().getGlobalPoseFor(Robot.m_robotContainer.turret).toPose2d();

        LimelightHelpers.SetRobotOrientation("limelight-turret", 
            currentTurretPose.getRotation().getDegrees(), 
            0, 0, 0, 0, 0);
        
    }

}
