// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;


/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private final DifferentialDrive m_robotDrive;
  private final Joystick m_driver;

  private final SparkMax m_leftFrontMotor = new SparkMax(4,MotorType.kBrushed);
  private final SparkMax m_rightFrontMotor = new SparkMax(5,MotorType.kBrushed);
  private final SparkMax m_leftRearMotor = new SparkMax(51,MotorType.kBrushed);
  private final SparkMax m_rightRearMotor = new SparkMax(3,MotorType.kBrushed);

  /** Called once at the beginning of the robot program. */
  public Robot() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_rightFrontMotor.setInverted(true);
    //m_rightRearMotor.setInverted(true);

    // Make the rears follow the fronts...
    //m_leftRearMotor.follow((m_leftFrontMotor));
    //m_rightRearMotor.follow((m_rightFrontMotor));
    m_leftFrontMotor.setInverted(true);
    m_leftRearMotor.setInverted(true);

    SparkMaxConfig leftConfig=new SparkMaxConfig();
    leftConfig.follow (m_leftRearMotor);
    m_leftFrontMotor.configure(leftConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    SparkMaxConfig rightConfig=new SparkMaxConfig();
    rightConfig.follow(m_rightRearMotor);
    m_rightFrontMotor.configure(rightConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    // Setup Drive and control systems
    m_robotDrive = new DifferentialDrive(m_leftRearMotor::set, m_rightRearMotor::set);

    m_driver = new Joystick(0);

    SendableRegistry.addChild(m_robotDrive, m_rightFrontMotor);
    SendableRegistry.addChild(m_robotDrive, m_leftFrontMotor);
  }

  @Override
  public void teleopPeriodic() {
    m_robotDrive.tankDrive(-m_driver.getRawAxis(1),-m_driver.getRawAxis(5));  }
}


//Of something, anything