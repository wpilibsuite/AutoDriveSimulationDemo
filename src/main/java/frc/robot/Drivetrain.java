/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N2;

@SuppressWarnings("PMD.TooManyFields")
public class Drivetrain {
  private static final double kWheelRadius = Units.inchesToMeters(2.0);
  private static final int kEncoderResolution = 4096;

  private final PWMVictorSPX m_leftLeader = new PWMVictorSPX(1);
  private final PWMVictorSPX m_leftFollower = new PWMVictorSPX(2);
  private final PWMVictorSPX m_rightLeader = new PWMVictorSPX(3);
  private final PWMVictorSPX m_rightFollower = new PWMVictorSPX(4);

  private final SpeedControllerGroup m_leftGroup
      = new SpeedControllerGroup(m_leftLeader, m_leftFollower);
  private final SpeedControllerGroup m_rightGroup
      = new SpeedControllerGroup(m_rightLeader, m_rightFollower);

  private final Encoder m_leftEncoder = new Encoder(0, 1);
  private final Encoder m_rightEncoder = new Encoder(2, 3);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final DifferentialDriveOdometry m_odometry
      = new DifferentialDriveOdometry(m_gyro.getRotation2d());

  // Simulation classes help us simulate our robot
  private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

  private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

  // Create the simulation model of our drivetrain.
  DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
    DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
    7.29,                    // 7.29:1 gearing reduction.
    7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
    60.0,                    // The mass of the robot is 60 kg.
    kWheelRadius,            // The robot uses 2" radius wheels.
    0.7112,                  // The track width is 0.7112 meters.
    null);

    private final Field2d m_field = new Field2d();

  public Drivetrain() {
    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    m_rightGroup.setInverted(true);

    m_leftEncoder.reset();
    m_rightEncoder.reset();

    SmartDashboard.putData("Field", m_field);
  }

  public void drivePercent(double left, double right) {
    m_leftGroup.set(left);
    m_rightGroup.set(right);
  }

  public void updateOdometry() {
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance());
  }

  public void resetOdometry(Pose2d pose) {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_driveSim.setPose(pose);
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  public double getDistanceMeters() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_driveSim.setInputs(m_leftLeader.get() * RobotController.getInputVoltage(),
        -m_rightLeader.get() * RobotController.getInputVoltage());
    m_driveSim.update(0.02);

    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());
  }

  public void periodic() {
    updateOdometry();
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }
}
