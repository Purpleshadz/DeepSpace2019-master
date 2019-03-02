/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void InitializeDashboard();
  void InitializePIDControllers();
  bool DoLimelightTracking();
  void ReadDashboard();
  void OperatorControl();

 private:
  static const int leftLeadDeviceID = 7, rightLeadDeviceID = 3, leftFollowDeviceID = 8, rightFollowDeviceID = 4;
  static const int armDeviceID = 1, wristDeviceID = 2, climbArmDeviceID = 5, climbFootDeviceID = 6;


  // Network Table
  std::shared_ptr<NetworkTable> m_limelightFront = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  std::shared_ptr<NetworkTable> m_limelightRear  = nt::NetworkTableInstance::GetDefault().GetTable("limelight-rear");

  // Targeting state
  bool m_IsTargeting = false;

  // Throttle values for target-tracking mode based on camera target offset
  double m_limelightTurnCmd = 0.0;
  double m_limelightDriveCmd = 0.0;
  
  // Pneumatics
  frc::DoubleSolenoid m_gearbox_right{3, 4};
  frc::DoubleSolenoid m_gearbox_left{2, 5};
  frc::DoubleSolenoid m_grabber{1, 6};

  frc::Compressor m_compressor;

  // Drive Motors
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  // Arm and Wrist Motors
  rev::CANSparkMax m_ArmMotor{armDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_wristMotor{wristDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  // Climber Motors
  rev::CANSparkMax m_climbArmMotor{climbArmDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_climbFootMotor{climbFootDeviceID, rev::CANSparkMax::MotorType::kBrushless};
 
  // Robot Drive
  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

  // Joystick
  frc::Joystick m_stick{0};

  // Encoders
  rev::CANEncoder m_armEncoder       = m_ArmMotor.GetEncoder();
  rev::CANEncoder m_wristEncoder     = m_wristMotor.GetEncoder();
  rev::CANEncoder m_climbArmEncoder  = m_climbArmMotor.GetEncoder();
  rev::CANEncoder m_climbFootEncoder = m_climbFootMotor.GetEncoder();

  // PID Controllers
  rev::CANPIDController m_armPidController       = m_ArmMotor.GetPIDController();
  rev::CANPIDController m_wristPidController     = m_wristMotor.GetPIDController();
  rev::CANPIDController m_climbArmPidController  = m_climbArmMotor.GetPIDController();
  rev::CANPIDController m_climbFootPidController = m_climbFootMotor.GetPIDController();

  // PID coefficient structure
  struct pidCoeff {
    double kP;
    double kI;
    double kD;
    double kIz;
    double kFF;
    double kMinOutput;
    double kMaxOutput;
  };

  // DETERMINE THESE EXPERIMENTALLY!!!!!!!
  pidCoeff       m_armCoeff {0.02, 0.0, 0.7, 0.0, 0.0, -1.0, 1.0};
  pidCoeff     m_wristCoeff {0.13, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0};
  pidCoeff m_climbFootCoeff {0.1, 0.0, 1.0, 0.0, 0.0, -1.0, 1.0};
  pidCoeff  m_climbArmCoeff {0.05, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0};

  // Set Points for arm/wrist positions
  double m_armRotations[4]    {0.0, 20.0, 49.0, 99.0};
  double m_wristRotations[4]  {0.0, 23.5, 40.0, 9.0};
  //double m_climbFootRotations = 0.0;
  double m_climbFootRotations = 132.0;
  //double m_climbArmRotations  = 0.0;
  double m_climbArmRotations  = 74.0;
  // Drive to get Robot on level
  double m_RobotDriveVelocity = .25;
  // Timer
  int timer = 0;
  int failSafe = 0;
  // Toggle for climb
  int climbToggle = 0;

  // // Cameras
  // cs::UsbCamera Cam1;
  // cs::UsbCamera Cam2;
};


