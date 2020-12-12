/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);

  private final Drivetrain m_drive = new Drivetrain();

  private final double kSetpoint = 3.0;

  private final PIDController m_pidController = new PIDController(10, 0, 0.0);

  @Override
  public void robotInit() {
    SmartDashboard.putNumber("Kp", 10.0);
    SmartDashboard.putNumber("Kd", 0.0);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Setpoint", kSetpoint);
    SmartDashboard.putNumber("Current Position", m_drive.getDistanceMeters());
    m_drive.periodic();
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    m_pidController.setP(SmartDashboard.getNumber("Kp", 10.0));
    m_pidController.setD(SmartDashboard.getNumber("Kd", 0.0));

    double output = m_pidController.calculate(m_drive.getDistanceMeters(), kSetpoint);
    m_drive.drivePercent(output, output);
  }

  @Override
  @SuppressWarnings("LocalVariableName")
  public void teleopPeriodic() {
    double xSpeed = -m_controller.getY(Hand.kLeft);
    double rot = -m_controller.getX(Hand.kLeft) * 0.4;

    m_drive.drivePercent(xSpeed - rot, xSpeed + rot);
  }

  @Override
  public void simulationPeriodic() {
    m_drive.simulationPeriodic();
  }

  @Override
  public void disabledInit() {
    m_drive.drivePercent(0.0, 0.0);
    m_drive.resetOdometry(new Pose2d(2, 2, new Rotation2d()));
  }
}
