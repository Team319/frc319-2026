package frc319.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc319.lib.io.ArticulatedComponent;
import frc319.lib.io.MotorIO;
import frc319.lib.subsystem.MotorFollowerSubsystem;
import frc319.lib.subsystem.TalonFXSubsystemConfig;
import frc319.lib.util.EqualsUtil;
import frc319.lib.util.RobotTime;
import frc319.lib.io.MotorInputsAutoLogged;
import frc319.robot.Constants;
import frc319.robot.FieldConstants;
import frc319.robot.subsystems.launcher.LauncherConstants.Flywheels.FlywheelsState;
import frc319.robot.subsystems.launcher.LaunchingSolutionManager.LaunchSolution;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends MotorFollowerSubsystem<MotorInputsAutoLogged, MotorIO>
    implements ArticulatedComponent {

  private FuelTrajectories fuelTrajectories = new FuelTrajectories();
  private Time lastUpdateTime = RobotTime.getTimestamp();


  private FlywheelsState flywheelsState = LauncherConstants.Flywheels.FlywheelsState.IDLE;

  // Elastic slider for testing flywheel speed (0 to 6000 RPM)
  private DoubleSupplier testFlywheelRPM = () -> 0.0;  // Default to 0, will be set from Elastic
  Distance toGoal = Meters.of(0.0);


  public Flywheels(
      final TalonFXSubsystemConfig leftConfig,
      final TalonFXSubsystemConfig rightConfig,
      final MotorIO leftLauncherMotorIO,
      final MotorIO rightLauncherMotorIO) {
    super(
        "Flywheel",
        leftConfig,
        rightConfig,
        new MotorInputsAutoLogged(),
        new MotorInputsAutoLogged(),
        leftLauncherMotorIO,
        rightLauncherMotorIO);
        this.fuelTrajectories = new FuelTrajectories();
  }

  public void setState(FlywheelsState newState) {
  this.flywheelsState = newState;
  }

  public Command setVelocity(Supplier<AngularVelocity> desiredVelocity) {
    return velocitySetpointCommand(desiredVelocity);
  }

  public Command stop() {
    return setVelocity(() -> RotationsPerSecond.of(0));
  }

  /**
   * Sets the supplier for the test flywheel RPM slider.
   * Call this from RobotContainer to connect an Elastic slider.
   * 
   * @param rpmSupplier A DoubleSupplier that returns the desired RPM from Elastic
   */
  public void setTestFlywheelRPMSupplier(DoubleSupplier rpmSupplier) {
    this.testFlywheelRPM = rpmSupplier;
  }

  @Override
  public void periodic() {
    Logger.recordOutput(pb.makePath("state"), flywheelsState);
    Logger.recordOutput(pb.makePath("curretFlywheelRPM"), super.getCurrentVelocity().in(RPM));


    super.periodic();
    
    //Logger.recordOutput(pb.makePath("flywheelVelocityRPM"), super.getLeftCurrentVelocity().in(RPM));

    var solution = LaunchingSolutionManager.getInstance().getSolution();

    switch (flywheelsState) {

      case SHOOT:
          if(Constants.useTurretLimelight){
            toGoal = LauncherVisionManager.getInstance().get2dDistanceToCurrentTarget();
          }
          else{
            toGoal = this.getDistance2d(LaunchingSolutionManager.getInstance().getTargetPose());
          }

        Logger.recordOutput(super.pb.makePath("distanceToGoal"), toGoal);
        CommandScheduler.getInstance().schedule(this.setVelocity(()->RPM.of(LauncherConstants.Flywheels.flywheelRPMMap.get(toGoal.in(Meters)))));
        

        // TODO :  need matching flywheel ANGULAR Velocity
        //if (solution.isValid()) {
          launchFuel(solution);
        //}
        break;

      case SHOOT_ON_MOVE:
        toGoal = this.getDistance2d(LaunchingSolutionManager.getInstance().getTargetOnMovePose());
        Logger.recordOutput(super.pb.makePath("distanceToGoal"), toGoal);
        CommandScheduler.getInstance().schedule(this.setVelocity(()->RPM.of(LauncherConstants.Flywheels.flywheelRPMMap.get(toGoal.in(Meters)))));
        

        // TODO :  need matching flywheel ANGULAR Velocity
        //if (solution.isValid()) {
          launchFuel(solution);
        //}
        break;

      case PRESPIN:
        CommandScheduler.getInstance().schedule(this.setVelocity(()->RPM.of(1800.0)));
        break;

      case DUMB_SHOT:
        toGoal = Meters.of(1.7);
        CommandScheduler.getInstance().schedule(this.setVelocity(() -> RPM.of(LauncherConstants.Flywheels.flywheelRPMMap.get(toGoal.in(Meters)))));
        break;

      case TUNING:
        // Use the value from Elastic slider
        //System.out.println("Test Flywheel RPM: " + testFlywheelRPM.getAsDouble()); // Debug print
        CommandScheduler.getInstance().schedule(this.setVelocity(() -> RPM.of(SmartDashboard.getNumber("Tuning_Mode/Flywheel_Tuning_RPM", 0.0))));
        break;

      case IDLE:
      default:
        CommandScheduler.getInstance().schedule(this.setVelocity(() -> RPM.of(0)));
        break;
    }

    Pose3d globalPose = this.getGlobalPose();
    Logger.recordOutput(
        super.pb.makePath("ball_vector"),
        new Pose3d[] {
          globalPose, globalPose.plus(new Transform3d(new Translation3d(1, 0, 0), new Rotation3d()))
        });

    Time now = RobotTime.getTimestamp();
    Time dt = now.minus(lastUpdateTime);
    this.fuelTrajectories.update(dt);
    this.lastUpdateTime = now;
    Logger.recordOutput(pb.makePath("fuel_trajectories"), this.fuelTrajectories.getPositions());

  }

  @Override
  public Transform3d getTransform3d() {
    return LauncherConstants.Flywheels.localTransform;
  }

  @Override
  public Translation3d getRelativeLinearVelocity() {
    return new Translation3d(0, 0, 0);
  }

  public LinearVelocity getSurfaceSpeed() {
    AngularVelocity wheelSpeed = getLeaderCurrentVelocity();
    Distance wheelDiameter = Inches.of(4);
    Distance wheelCircumference = wheelDiameter.times(Math.PI);
    return InchesPerSecond.of(wheelSpeed.in(RotationsPerSecond) * wheelCircumference.in(Inches));
  }

  public LinearVelocity getLaunchVelocity() {
    Distance toGoal = this.getDistance2d(FieldConstants.Hub.innerCenterPoint);
    LinearVelocity vel =
        FeetPerSecond.of(LauncherConstants.Flywheels.velocityMap.get(toGoal.in(Meters)));
    Logger.recordOutput(pb.makePath("launchVelocity"), vel);
    return vel;
  }

  public LinearVelocity getOnTheFlyLaunchVelocity(LaunchSolution solution) {

    if (solution.isValid()) {
      double targetSpeedMps = solution.flywheelSpeedMetersPerSecond();
      Logger.recordOutput(super.pb.makePath("launchVelocity"), targetSpeedMps);
      return MetersPerSecond.of(targetSpeedMps);
    }
    return MetersPerSecond.of(0);
  }

  public Translation3d getLaunchVector(LaunchSolution solution) {
    // A. Robot Structure Velocity (Drive + Turret Spin + Hood Pitch)
    // If the robot is driving 2m/s North, this returns (0, 2, 0)
    Translation3d structureVel = this.getGlobalLinearVelocity();

    // B. Muzzle Velocity (Shot Power)

    // Define the launch speed relative to the flywheel (Forward X)
    Translation3d flywheelVelRel =
        new Translation3d(getOnTheFlyLaunchVelocity(solution).in(MetersPerSecond), 0, 0);

    // C. Rotate Muzzle Velocity to Global Frame
    // We use the Global Rotation of the flywheels (which includes Drive, Turret, Hood)
    Rotation3d launcherFacing = this.getGlobalPose().getRotation();
    Translation3d launchVelGlobal = flywheelVelRel.rotateBy(launcherFacing);

    // D. Combine: V_ball = V_robot + V_shot
    return structureVel.plus(launchVelGlobal);
  }

  public boolean isAtTargetVelocity() {
    double currentRPM = super.getCurrentVelocity().in(RPM);
   // Distance toGoal ;//= this.getDistance2d(LaunchingSolutionManager.getInstance().getTargetPose());
    double targetRPM = LauncherConstants.Flywheels.flywheelRPMMap.get(toGoal.in(Meters));
    AngularVelocity targetVelocity = RPM.of(targetRPM) ;
    Logger.recordOutput(pb.makePath("targetFlywheelRPM"), targetVelocity);
    Logger.recordOutput(pb.makePath("currentFlywheelRPM"), currentRPM);
    return  currentRPM >= targetRPM - 100.0;//EqualsUtil.epsilonEquals(currentRPM, targetRPM, 250); 
  }

  // EKM - Is this just simulated???
  public void launchFuel(LaunchSolution solution) {
    this.fuelTrajectories.launch(
        this.getGlobalPose().getTranslation(), getLaunchVector(solution), RotationsPerSecond.of(0));
  }
}
