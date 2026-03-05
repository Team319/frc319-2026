package frc319.lib.subsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc319.lib.io.AdvantageScopePathBuilder;
import frc319.lib.io.MotorIO;
import frc319.lib.util.RobotTime;
import frc319.lib.util.Util;
import frc319.lib.io.MotorInputsAutoLogged;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * A subsystem that manages two motors, where one follows the other. The left motor is the leader
 * and the right motor is the follower.
 */
public class MotorFollowerSubsystem<MI extends MotorInputsAutoLogged, IO extends MotorIO>
    extends MotorSubsystem<MI, IO> {
  protected final IO followerIO;
  protected final MI followerInputs;
  protected final TalonFXSubsystemConfig followerConfig;

  protected Angle positionSetpoint = Radians.of(0.0);
  protected AngularVelocity velocitySetpoint = RotationsPerSecond.of(0.0);

  public MotorFollowerSubsystem(
      String name,
      TalonFXSubsystemConfig leaderConfig,
      TalonFXSubsystemConfig followerConfig,
      MI leaderInputs,
      MI followerInputs,
      IO leaderIO,
      IO followerIO) {
    // Set the subsystem name
    super(leaderConfig, leaderInputs, leaderIO);
    setName(name);
    this.followerConfig = followerConfig;
    this.followerInputs = followerInputs;
    this.followerIO = followerIO;


    // Set the default command to stop both motors
    setDefaultCommand(
        this.dutyCycleCommand(() -> 0.0)
            .withName(pb.makeName("DefaultCommand"))
            .ignoringDisable(true));

    this.followerIO.follow(config.talonCANID, true);
  }

  @Override
  public void periodic() {
    super.periodic();
    Time timestamp = RobotTime.getTimestamp();
    followerIO.readInputs(followerInputs);
    Logger.processInputs(getName() + "/Follower", followerInputs);
    Logger.recordOutput(pb.makePath("LatencyPeriodSec"), RobotTime.getTimestamp().minus(timestamp));
    Logger.recordOutput(
        pb.makePath("currentCommand"),
        (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());

    // Log setpoints for comparison with measurements (in same units as inputs)
    Logger.recordOutput(
        pb.makePath("Setpoints", "velocityRotPerSec"), velocitySetpoint.in(RotationsPerSecond));
    Logger.recordOutput(pb.makePath("Setpoints", "positionRot"), positionSetpoint.in(Rotations));
    // Also log measurements as doubles for easy comparison
    Logger.recordOutput(
        pb.makePath("Measurements", "leaderVelocityRotPerSec"),
        inputs.velocity.in(RotationsPerSecond));
    Logger.recordOutput(
        pb.makePath("Measurements", "followerVelocityRotPerSec"),
        followerInputs.velocity.in(RotationsPerSecond));
    Logger.recordOutput(
        pb.makePath("Measurements", "leaderAppliedVolts"), inputs.appliedVolts.in(Volts));
    Logger.recordOutput(
        pb.makePath("Measurements", "followerAppliedVolts"), followerInputs.appliedVolts.in(Volts));
    Logger.recordOutput(pb.makePath("Measurements", "closedLoopError"), inputs.closedLoopError);
  }

  // Getters
  /**
   * Gets the current position of the leader motor.
   *
   * @return The current position of the leader motor.
   */
  public Angle getLeaderCurrentPosition() {
    return inputs.position;
  }

  /**
   * Gets the current position of the follower motor.
   *
   * @return The current position of the follower motor.
   */
  public Angle getFollowerCurrentPosition() {
    return followerInputs.position;
  }

  /**
   * Gets the current velocity of the leader motor.
   *
   * @return The current velocity of the leader motor.
   */
  public AngularVelocity getLeaderCurrentVelocity() {
    return inputs.velocity;
  }

  /**
   * Gets the current velocity of the follower motor.
   *
   * @return The current velocity of the follower motor.
   */
  public AngularVelocity getFollowerCurrentVelocity() {
    return followerInputs.velocity;
  }

  // Command Generators - Most commands are inherited from MotorSubsystem
  // Only follower-specific commands that need to check both motors are defined here

  /**
   * Creates a command that sets both motors to the specified position setpoint and ends when both
   * positions are within the specified epsilon of the target.
   *
   * @param positionSupplier The desired position setpoint supplier.
   * @param epsilonSupplier The acceptable error margin supplier.
   * @return A command that sets both motors to the specified position setpoint and ends when on
   *     target.
   */
  public Command positionSetpointUntilOnTargetCommand(
      Supplier<Angle> positionSupplier, Supplier<Angle> epsilonSupplier) {
    return new ParallelDeadlineGroup(
            new WaitUntilCommand(
                () ->
                    Util.epsilonEquals(
                            positionSupplier.get(), inputs.position, epsilonSupplier.get())
                        && Util.epsilonEquals(
                            positionSupplier.get(),
                            followerInputs.position,
                            epsilonSupplier.get())),
            positionSetpointCommand(positionSupplier))
        .withName(pb.makeName("PositionUntilOnTargetControl"));
  }

  /**
   * Creates a command that sets both motors to the specified velocity setpoint and ends when both
   * velocities are within the specified epsilon of the target.
   *
   * @param velocitySupplier The desired velocity setpoint supplier.
   * @param epsilonSupplier The acceptable error margin supplier.
   * @return A command that sets both motors to the specified velocity setpoint and ends when on
   *     target.
   */
  public Command velocitySetpointUntilOnTargetCommand(
      Supplier<AngularVelocity> velocitySupplier, Supplier<AngularVelocity> epsilonSupplier) {
    return new ParallelDeadlineGroup(
            new WaitUntilCommand(
                () ->
                    Util.epsilonEquals(
                            velocitySupplier.get(), inputs.velocity, epsilonSupplier.get())
                        && Util.epsilonEquals(
                            velocitySupplier.get(),
                            followerInputs.velocity,
                            epsilonSupplier.get())),
            velocitySetpointCommand(velocitySupplier))
        .withName(pb.makeName("VelocityUntilOnTargetControl"));
  }

  /**
   * Creates a command that sets both motors to the specified position setpoint using Motion Magic
   * control and ends when both positions are within the specified tolerance of the target.
   *
   * @param setpointSupplier The desired position setpoint supplier.
   * @param tolerance The acceptable error margin.
   * @param slot The PID slot to use for the Motion Magic control.
   * @return A command that sets both motors to the specified position setpoint using Motion Magic
   *     control and ends when on target.
   */
  public Command motionMagicSetpointCommandBlocking(
      Supplier<Angle> setpointSupplier, Angle tolerance, int slot) {
    return motionMagicSetpointCommand(setpointSupplier, slot)
        .until(
            () ->
                Util.epsilonEquals(getLeaderCurrentPosition(), setpointSupplier.get(), tolerance)
                    && Util.epsilonEquals(
                        getFollowerCurrentPosition(), setpointSupplier.get(), tolerance));
  }

  /**
   * Creates a command that sets both motors to the specified position setpoint using Motion Magic
   * control with dynamic parameters and ends when both positions are within the specified tolerance
   * of the target.
   *
   * @param setpointSupplier The desired position setpoint supplier.
   * @param configSupplier The Motion Magic configuration supplier.
   * @param tolerance The acceptable error margin.
   * @param slot The PID slot to use for the Motion Magic control.
   * @return A command that sets both motors to the specified position setpoint using Motion Magic
   *     control with dynamic parameters and ends when on target.
   */
  public Command motionMagicSetpointCommandBlocking(
      Supplier<Angle> setpointSupplier,
      Supplier<MotionMagicConfigs> configSupplier,
      Angle tolerance,
      int slot) {
    return motionMagicSetpointCommand(setpointSupplier, configSupplier, slot)
        .until(
            () ->
                Util.epsilonEquals(getLeaderCurrentPosition(), setpointSupplier.get(), tolerance)
                    && Util.epsilonEquals(
                        getFollowerCurrentPosition(), setpointSupplier.get(), tolerance));
  }

  /**
   * Creates a command that sets both motors to the specified position setpoint using Motion Magic
   * control with dynamic parameters and feedforward, and ends when both positions are within the
   * specified tolerance of the target.
   *
   * @param setpointSupplier The desired position setpoint supplier.
   * @param configSupplier The Motion Magic configuration supplier.
   * @param feedforward The feedforward supplier.
   * @param tolerance The acceptable error margin.
   * @param slot The PID slot to use for the Motion Magic control.
   * @return A command that sets both motors to the specified position setpoint using Motion Magic
   *     control with dynamic parameters and feedforward, and ends when on target.
   */
  public Command motionMagicSetpointCommandBlocking(
      Supplier<Angle> setpointSupplier,
      Supplier<MotionMagicConfigs> configSupplier,
      Supplier<Double> feedforward,
      Angle tolerance,
      int slot) {
    return motionMagicSetpointCommand(setpointSupplier, configSupplier, feedforward, slot)
        .until(
            () ->
                Util.epsilonEquals(getLeaderCurrentPosition(), setpointSupplier.get(), tolerance)
                    && Util.epsilonEquals(
                        getFollowerCurrentPosition(), setpointSupplier.get(), tolerance));
  }

  /**
   * Creates a command that temporarily disables software limits while running.
   *
   * @return A command that temporarily disables software limits while running.
   */
  protected Command withoutLimitsTemporarily() {
    var prevLeader =
        new Object() {
          boolean fwd = false;
          boolean rev = false;
        };

    return Commands.startEnd(
        () -> {
          prevLeader.fwd = config.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable;
          prevLeader.rev = config.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable;

          io.setEnableSoftLimits(false, false);
        },
        () -> {
          io.setEnableSoftLimits(prevLeader.fwd, prevLeader.rev);
        });
  }
}
