/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "log.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>  
#include <frc/WPILib.h>

double clamp(double in, double minval, double maxval)
{
  if (in > maxval) return maxval;
  if (in < minval) return minval;
  return in;
}

void Robot::RobotInit() {

  m_compressor.Start();
	m_gearbox_right.Set(frc::DoubleSolenoid::Value::kOff);
	m_gearbox_left.Set(frc::DoubleSolenoid::Value::kOff);
	m_grabber.Set(frc::DoubleSolenoid::Value::kOff);

  m_leftFollowMotor.Follow(m_leftLeadMotor);
  m_rightFollowMotor.Follow(m_rightLeadMotor);

  InitializeDashboard();
  InitializePIDControllers();

}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as wel
 * l.
 */
void Robot::AutonomousInit() {
  ReadDashboard();
  m_armPidController.SetReference(m_armRotations[0], rev::ControlType::kPosition);
  m_wristPidController.SetReference(m_wristRotations[0], rev::ControlType::kPosition);
  m_climbArmPidController.SetReference(0, rev::ControlType::kPosition);
}

void Robot::AutonomousPeriodic() {
  OperatorControl();
}

void Robot::TeleopInit() {
  ReadDashboard();
}

void Robot::TeleopPeriodic() {
  OperatorControl();
}

void Robot::OperatorControl()
{
  // Drive input from joystick
  double move   = .5*m_stick.GetRawAxis(1);
  double rotate = 0.75*m_stick.GetRawAxis(4);
  // Targeting or Driving?
  if (m_stick.GetRawAxis(3) > 0.75) {
    // Targeting
    m_limelightFront->PutNumber("pipeline", 0);
    m_limelightRear->PutNumber("pipeline", 0);
    m_IsTargeting = true;

    DoLimelightTracking();
    move   = m_limelightDriveCmd;
    rotate = m_limelightTurnCmd;
  // if grabber is closed
    if (m_grabber.Get() == frc::DoubleSolenoid::Value::kReverse) {
      if ((switch1.Get() == 0) || (switch0.Get() == 0) || (switch2.Get() == 0) || (switch3.Get() == 0)){
        m_grabber.Set(frc::DoubleSolenoid::Value::kForward);
        m_IsStopped = true;
        LOGGER(INFO) << "Stopped";
      }
    }
    if (m_IsStopped){
      //zero motor input
      move = 0.0;
      rotate = 0.0;
    }
  } else {
    // Driving
    if (m_IsTargeting) {
      // Stopped targeting, toggle grabber
      m_IsTargeting = false;
      if(!m_IsStopped == true){
        if(m_grabber.Get() == frc::DoubleSolenoid::Value::kForward){
          m_grabber.Set(frc::DoubleSolenoid::Value::kReverse);
        }
      }
    m_IsStopped = false;
  }
    m_limelightFront->PutNumber("pipeline", 1);
    m_limelightRear->PutNumber("pipeline", 1);

    // Deadband
    if (fabs(move) < 0.15) {
      move = 0.0;
    }

    if (fabs(rotate) < 0.15) {
      rotate = 0.0;
    }
  }
  // LOGGER(INFO) << switch0.Get() << " switch0";
  // LOGGER(INFO) << switch1.Get() << " switch1";
  // LOGGER(INFO) << switch2.Get() << " switch2";
  // LOGGER(INFO) << switch3.Get() << " switch3";
  // Robot Drive
  m_robotDrive.ArcadeDrive(move, rotate);

  // Shifting
  if (m_stick.GetRawButton(5)) {
    LOGGER(INFO) << "High Gear";
		m_gearbox_right.Set(frc::DoubleSolenoid::Value::kForward);
    m_gearbox_left.Set(frc::DoubleSolenoid::Value::kForward);
  } else if (m_stick.GetRawButton(6)) {
    LOGGER(INFO) << "Low Gear";
		m_gearbox_right.Set(frc::DoubleSolenoid::Value::kReverse);
		m_gearbox_left.Set(frc::DoubleSolenoid::Value::kReverse);
	}

 // Grabber
 if (m_stick.GetRawButton(7)) {
  //  LOGGER(INFO) << "Grabber Open";
		m_grabber.Set(frc::DoubleSolenoid::Value::kForward);
  } else if (m_stick.GetRawButton(8)) {
  //  LOGGER(INFO) << "Grabber Close";
		m_grabber.Set(frc::DoubleSolenoid::Value::kReverse);
	} 

if (m_stick.GetRawAxis(2) > 0.5){
  failSafe++;
}
else{
  failSafe = 0;
}
if(failSafe >= 50){    
  if(m_stick.GetRawAxis(2) > 0.5 && (m_climbFootEncoder.GetPosition() <= 132)){
    m_climbArmPidController.SetReference(m_climbArmRotations, rev::ControlType::kPosition);

    if(m_climbArmEncoder.GetPosition() >= 65) {
      m_climbFootPidController.SetReference(m_climbFootRotations, rev::ControlType::kPosition);
}

}

  if(m_stick.GetRawAxis(2) > 0.5 && (m_climbFootEncoder.GetPosition() > 65)){
    m_climbFootPidController.SetReference(m_climbFootRotations, rev::ControlType::kPosition);
    m_climbArmPidController.SetReference(0, rev::ControlType::kPosition);
}
  if((m_climbFootEncoder.GetPosition() <= 125) && (m_climbArmEncoder.GetPosition() >= 0) && (timer >= 85)){
    timer++;
    m_gearbox_right.Set(frc::DoubleSolenoid::Value::kReverse);
	  m_gearbox_left.Set(frc::DoubleSolenoid::Value::kReverse);
    m_robotDrive.ArcadeDrive(m_RobotDriveVelocity, 0);
  }
}


// //Level 1 Climb
//   if ((m_stick.GetRawButton(9)) && (m_stick.GetRawButton(10))){
//     timerArm++;
//   }
//   if ((m_stick.GetRawButton(9)) && (m_stick.GetRawButton(10)) && (timerArm >= 85)){
//     m_climbArmPidController.SetReference(m_climbArmRotationsLowLevel, rev::ControlType::kPosition);
    
//     if ((m_climbArmEncoder.GetPosition() >= 80) && (timerArmDrive <= 85)) {
//       timerArmDrive++;
//       m_gearbox_right.Set(frc::DoubleSolenoid::Value::kReverse);
// 	    m_gearbox_left.Set(frc::DoubleSolenoid::Value::kReverse);
//       m_robotDrive.ArcadeDrive(m_RobotDriveVelocity, 0);
//     }
    
//     if (timerArmDrive >= 60){
//     m_climbArmPidController.SetReference(0, rev::ControlType::kPosition);
//     }
//   }
  // Arm/Wrist Positioning
  if (m_stick.GetRawButton(1)) {
    m_armPidController.SetReference(m_armRotations[0], rev::ControlType::kPosition);
    m_wristPidController.SetReference(m_wristRotations[0], rev::ControlType::kPosition);
    //LOGGER(INFO) << "Level 0 " << m_wristRotations[0];
  } else if (m_stick.GetRawButton(2)) {
    m_armPidController.SetReference(m_armRotations[1], rev::ControlType::kPosition);
    m_wristPidController.SetReference(m_wristRotations[1], rev::ControlType::kPosition);
    //LOGGER(INFO) << "Level 1 " << m_wristRotations[1];
  } else if (m_stick.GetRawButton(3)) {
    m_armPidController.SetReference(m_armRotations[2], rev::ControlType::kPosition);
    m_wristPidController.SetReference(m_wristRotations[2], rev::ControlType::kPosition);
    //LOGGER(INFO) << "Level 2 " << m_wristRotations[2];
  } else if (m_stick.GetRawButton(4)) {
    m_armPidController.SetReference(m_armRotations[3], rev::ControlType::kPosition);
    m_wristPidController.SetReference(m_wristRotations[3], rev::ControlType::kPosition);
    //LOGGER(INFO) << "Level 3 " << m_wristRotations[3];
  }

  // if ((m_stick.GetPOV(0) == 90) || (m_stick.GetPOV(0) == 270) && (m_POVClick == false)){
  //   m_POVClick = true;
  // }
  if (m_stick.GetPOV(0) == -1){
    m_POVClick = false;
  }

  if ((m_stick.GetPOV(0) == 90) && (m_POVClick == false)){
    m_wristRotations[1] = m_wristRotations[1] + 1;
    m_wristPidController.SetReference(m_wristRotations[1], rev::ControlType::kPosition);
    m_POVClick = true;
    LOGGER(INFO) << "Starting: " <<m_climbWristRotationlvl1 << " current: " << m_wristRotations[1];

  }
  if ((m_stick.GetPOV(0) == 270)  && (m_POVClick == false)){
    m_wristRotations[1] = m_wristRotations[1] - 1;
    m_wristPidController.SetReference(m_wristRotations[1], rev::ControlType::kPosition);
    m_POVClick = true;
    LOGGER(INFO) << "Starting: " <<m_climbWristRotationlvl1 << " current: " << m_wristRotations[1];

  }
  /*LOGGER(INFO) << "  Arm Encoder: " << m_armEncoder.GetPosition();
  LOGGER(INFO) << "Wrist Encoder: " << m_wristEncoder.GetPosition();
  LOGGER(INFO) << "  C/A Encoder: " << m_climbArmEncoder.GetPosition();
  LOGGER(INFO) << "  C/F Encoder: " << m_climbFootEncoder.GetPosition();*/
  if (m_stick.GetPOV(0) == 180){
    timerFootBall++;
  }
  if ((m_stick.GetPOV(0) == 180) && (timerFootBall >= 50)){
    m_climbFootPidController.SetReference(17, rev::ControlType::kPosition);
  }
  if ((m_stick.GetPOV(0) == 0) && (timerFootBall >= 50)){
    m_climbFootPidController.SetReference(0, rev::ControlType::kPosition);
  }
}
                                       

void Robot::TestPeriodic() {}
bool Robot::DoLimelightTracking()
{
  // Proportional Steering Constant:
  const double STEER_K = 0.1;
  const double MAX_STEER = 0.5;
  const double DRIVE_K = 0.55;

  // Front camera
  double txFront = m_limelightFront->GetNumber("tx", 0.0); // horizontal offset
  double tvFront = m_limelightFront->GetNumber("tv", 0.0); // target valid?
  double taFront = m_limelightFront->GetNumber("ta", 0.0); // target area
  // Rear camera
  double txRear  = m_limelightRear->GetNumber("tx", 0.0); // horizontal offset
  double tvRear  = m_limelightRear->GetNumber("tv", 0.0); // target valid?
  double taRear  = m_limelightRear->GetNumber("ta", 0.0); // target area

  m_limelightTurnCmd  = 0.0;
  m_limelightDriveCmd = 0.0;

  if ((tvFront > 0.0) && (tvRear < 1.0)) {
    // Front target only detected
    m_limelightDriveCmd = -DRIVE_K;
    m_limelightTurnCmd  = txFront*STEER_K;
    m_limelightTurnCmd  = clamp(m_limelightTurnCmd, -MAX_STEER, MAX_STEER);
    //LOGGER(INFO) << "Target at Front";
    return true;

  } else if ((tvFront < 1.0) && (tvRear > 0.0)) {
    // Rear target only detected
    m_limelightDriveCmd = DRIVE_K;
    m_limelightTurnCmd  = txRear*STEER_K;
    m_limelightTurnCmd  = clamp(m_limelightTurnCmd, -MAX_STEER, MAX_STEER);
    //LOGGER(INFO) << "Target at Rear";
    return true;

  } else if ((tvFront > 0.0) && (tvRear > 0.0)) {
    // Front and Rear targets detected, compare areas
    if (taFront > taRear) {
      m_limelightDriveCmd = -DRIVE_K;
      m_limelightTurnCmd  = txFront*STEER_K;
      m_limelightTurnCmd  = clamp(m_limelightTurnCmd, -MAX_STEER, MAX_STEER);
      //LOGGER(INFO) << "Target at Front is larger";
    } else {
      m_limelightDriveCmd = DRIVE_K;
      m_limelightTurnCmd  = txRear*STEER_K;
      m_limelightTurnCmd  = clamp(m_limelightTurnCmd, -MAX_STEER, MAX_STEER);
      //LOGGER(INFO) << "Target at Rear is larger";
    }
    return true;
  } 

  //LOGGER(INFO) << "No Targets";
  return false;
}

void Robot::InitializePIDControllers() {
  m_armPidController.SetP(m_armCoeff.kP);
  m_armPidController.SetI(m_armCoeff.kI);
  m_armPidController.SetD(m_armCoeff.kD);
  m_armPidController.SetIZone(m_armCoeff.kIz);
  m_armPidController.SetFF(m_armCoeff.kFF);
  m_armPidController.SetOutputRange(m_armCoeff.kMinOutput, m_armCoeff.kMaxOutput);

  m_wristPidController.SetP(m_wristCoeff.kP);
  m_wristPidController.SetI(m_wristCoeff.kI);
  m_wristPidController.SetD(m_wristCoeff.kD);
  m_wristPidController.SetIZone(m_wristCoeff.kIz);
  m_wristPidController.SetFF(m_wristCoeff.kFF);
  m_wristPidController.SetOutputRange(m_wristCoeff.kMinOutput, m_wristCoeff.kMaxOutput);

  m_climbArmPidController.SetP(m_climbArmCoeff.kP);
  m_climbArmPidController.SetI(m_climbArmCoeff.kI);
  m_climbArmPidController.SetD(m_climbArmCoeff.kD);
  m_climbArmPidController.SetIZone(m_climbArmCoeff.kIz);
  m_climbArmPidController.SetFF(m_climbArmCoeff.kFF);
  m_climbArmPidController.SetOutputRange(m_climbArmCoeff.kMinOutput, m_climbArmCoeff.kMaxOutput);

  m_climbFootPidController.SetP(m_climbFootCoeff.kP);
  m_climbFootPidController.SetI(m_climbFootCoeff.kI);
  m_climbFootPidController.SetD(m_climbFootCoeff.kD);
  m_climbFootPidController.SetIZone(m_climbFootCoeff.kIz);
  m_climbFootPidController.SetFF(m_climbFootCoeff.kFF);
  m_climbFootPidController.SetOutputRange(m_climbFootCoeff.kMinOutput, m_climbFootCoeff.kMaxOutput);
}

void Robot::InitializeDashboard() {
  // display PID coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("Arm P Gain",              m_armCoeff.kP);
  frc::SmartDashboard::PutNumber("Arm I Gain",              m_armCoeff.kI);
  frc::SmartDashboard::PutNumber("Arm D Gain",              m_armCoeff.kD);
  frc::SmartDashboard::PutNumber("Arm Max Output",          m_armCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Arm Min Output",          m_armCoeff.kMinOutput);

  frc::SmartDashboard::PutNumber("Wrist P Gain",            m_wristCoeff.kP);
  frc::SmartDashboard::PutNumber("Wrist I Gain",            m_wristCoeff.kI);
  frc::SmartDashboard::PutNumber("Wrist D Gain",            m_wristCoeff.kD);
  frc::SmartDashboard::PutNumber("Wrist Max Output",        m_wristCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Wrist Min Output",        m_wristCoeff.kMinOutput);

  frc::SmartDashboard::PutNumber("Climb Arm P Gain",        m_climbArmCoeff.kP);
  frc::SmartDashboard::PutNumber("Climb Arm I Gain",        m_climbArmCoeff.kI);
  frc::SmartDashboard::PutNumber("Climb Arm D Gain",        m_climbArmCoeff.kD);
  frc::SmartDashboard::PutNumber("Climb Arm Max Output",    m_climbArmCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Climb Arm Min Output",    m_climbArmCoeff.kMinOutput);
  
  frc::SmartDashboard::PutNumber("Climb Foot P Gain",       m_climbFootCoeff.kP);
  frc::SmartDashboard::PutNumber("Climb Foot I Gain",       m_climbFootCoeff.kI);
  frc::SmartDashboard::PutNumber("Climb Foot D Gain",       m_climbFootCoeff.kD);
  frc::SmartDashboard::PutNumber("Climb Foot Max Output",   m_climbFootCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Climb Foot Min Output",   m_climbFootCoeff.kMinOutput);
  
  frc::SmartDashboard::PutNumber("Arm Rotations Level 0",   m_armRotations[0]);
  frc::SmartDashboard::PutNumber("Arm Rotations Level 1",   m_armRotations[1]);
  frc::SmartDashboard::PutNumber("Arm Rotations Level 2",   m_armRotations[2]);
  frc::SmartDashboard::PutNumber("Arm Rotations Level 3",   m_armRotations[3]);

  frc::SmartDashboard::PutNumber("Wrist Rotations Level 0", m_wristRotations[0]);
  frc::SmartDashboard::PutNumber("Wrist Rotations Level 1", m_wristRotations[1]);
  frc::SmartDashboard::PutNumber("Wrist Rotations Level 2", m_wristRotations[2]);
  frc::SmartDashboard::PutNumber("Wrist Rotations Level 3", m_wristRotations[3]);

  frc::SmartDashboard::PutNumber("Climb Foot Rotations",    m_climbFootRotations);
  frc::SmartDashboard::PutNumber("Climb Arm Rotations",     m_climbArmRotations);

  frc::SmartDashboard::PutNumber("Climb Drive Forward",     m_RobotDriveVelocity);
}

void Robot::ReadDashboard () {
  double p, i, d, min, max;

  // read PID coefficients from SmartDashboard
  p   = frc::SmartDashboard::GetNumber("Arm P Gain", 0);
  i   = frc::SmartDashboard::GetNumber("Arm I Gain", 0);
  d   = frc::SmartDashboard::GetNumber("Arm D Gain", 0);
  min = frc::SmartDashboard::GetNumber("Arm Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Arm Max Output", 0);

  // If PID coefficients on SmartDashboard have changed, write new values to controller
  if ((p != m_armCoeff.kP)) { m_armPidController.SetP(p); m_armCoeff.kP = p; }
  if ((i != m_armCoeff.kI)) { m_armPidController.SetI(i); m_armCoeff.kI = i; }
  if ((d != m_armCoeff.kD)) { m_armPidController.SetD(d); m_armCoeff.kD = d; }
  if ((max != m_armCoeff.kMaxOutput) || (min != m_armCoeff.kMinOutput)) { 
    m_armPidController.SetOutputRange(min, max); 
    m_armCoeff.kMinOutput = min; m_armCoeff.kMaxOutput = max; 
  }

  // read PID coefficients from SmartDashboard
  p   = frc::SmartDashboard::GetNumber("Wrist P Gain", 0);
  i   = frc::SmartDashboard::GetNumber("Wrist I Gain", 0);
  d   = frc::SmartDashboard::GetNumber("Wrist D Gain", 0);
  min = frc::SmartDashboard::GetNumber("Wrist Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Wrist Max Output", 0);

  // If PID coefficients on SmartDashboard have changed, write new values to controller
  if ((p != m_wristCoeff.kP)) { m_wristPidController.SetP(p); m_wristCoeff.kP = p; }
  if ((i != m_wristCoeff.kI)) { m_wristPidController.SetI(i); m_wristCoeff.kI = i; }
  if ((d != m_wristCoeff.kD)) { m_wristPidController.SetD(d); m_wristCoeff.kD = d; }
  if ((max != m_wristCoeff.kMaxOutput) || (min != m_wristCoeff.kMinOutput)) { 
    m_wristPidController.SetOutputRange(min, max); 
    m_wristCoeff.kMinOutput = min; m_wristCoeff.kMaxOutput = max; 
  }

  // read PID coefficients from SmartDashboard
  p   = frc::SmartDashboard::GetNumber("Climb Foot P Gain", 0);
  i   = frc::SmartDashboard::GetNumber("Climb Foot I Gain", 0);
  d   = frc::SmartDashboard::GetNumber("Climb Foot D Gain", 0);
  min = frc::SmartDashboard::GetNumber("Climb Foot Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Climb Foot Max Output", 0);

  // If PID coefficients on SmartDashboard have changed, write new values to controller
  if ((p != m_climbFootCoeff.kP)) { m_climbFootPidController.SetP(p); m_climbFootCoeff.kP = p; }
  if ((i != m_climbFootCoeff.kI)) { m_climbFootPidController.SetI(i); m_climbFootCoeff.kI = i; }
  if ((d != m_climbFootCoeff.kD)) { m_climbFootPidController.SetD(d); m_climbFootCoeff.kD = d; }
  if ((max != m_climbFootCoeff.kMaxOutput) || (min != m_climbFootCoeff.kMinOutput)) { 
    m_climbFootPidController.SetOutputRange(min, max); 
    m_climbFootCoeff.kMinOutput = min; m_climbFootCoeff.kMaxOutput = max; 
  }

  // read PID coefficients from SmartDashboard
  p   = frc::SmartDashboard::GetNumber("Climb Arm P Gain", 0);
  i   = frc::SmartDashboard::GetNumber("Climb Arm I Gain", 0);
  d   = frc::SmartDashboard::GetNumber("Climb Arm D Gain", 0);
  min = frc::SmartDashboard::GetNumber("Climb Arm Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Climb Arm Max Output", 0);

  // If PID coefficients on SmartDashboard have changed, write new values to controller
  if ((p != m_climbArmCoeff.kP)) { m_climbArmPidController.SetP(p); m_climbArmCoeff.kP = p; }
  if ((i != m_climbArmCoeff.kI)) { m_climbArmPidController.SetI(i); m_climbArmCoeff.kI = i; }
  if ((d != m_climbArmCoeff.kD)) { m_climbArmPidController.SetD(d); m_climbArmCoeff.kD = d; }
  if ((max != m_climbArmCoeff.kMaxOutput) || (min != m_climbArmCoeff.kMinOutput)) { 
    m_climbArmPidController.SetOutputRange(min, max); 
    m_climbArmCoeff.kMinOutput = min; m_climbArmCoeff.kMaxOutput = max; 
  }

  m_armRotations[0]    = frc::SmartDashboard::GetNumber("Arm Rotations Level 0", 0);
  m_armRotations[1]    = frc::SmartDashboard::GetNumber("Arm Rotations Level 1", 0);
  m_armRotations[2]    = frc::SmartDashboard::GetNumber("Arm Rotations Level 2", 0);
  m_armRotations[3]    = frc::SmartDashboard::GetNumber("Arm Rotations Level 3", 0);
  m_wristRotations[0]  = frc::SmartDashboard::GetNumber("Wrist Rotations Level 0", 0);
  m_wristRotations[1]  = frc::SmartDashboard::GetNumber("Wrist Rotations Level 1", 0);
  // m_wristRotations[1]  = m_climbWristRotationlvl1;
  m_wristRotations[2]  = frc::SmartDashboard::GetNumber("Wrist Rotations Level 2", 0);
  m_wristRotations[3]  = frc::SmartDashboard::GetNumber("Wrist Rotations Level 3", 0);

  m_climbFootRotations = frc::SmartDashboard::GetNumber("Climb Foot Rotations", 130);
  m_climbArmRotations  = frc::SmartDashboard::GetNumber("Climb Arm Rotations", 0);
  //m_climbArmRotations  = frc::SmartDashboard::GetNumber("Climb Arm Rotations", 74);

  m_RobotDriveVelocity = frc::SmartDashboard::GetNumber("Climb Drive Forward", 0);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
