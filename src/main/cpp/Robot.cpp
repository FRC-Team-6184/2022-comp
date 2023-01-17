// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
int idport=3;

bool play = false;

float motorstart;
float vert= 0;
float horz=0;
float pivot=0;
float servopos=0;
units::time::second_t startime;
units::time::second_t loader  ;

void Robot::RobotInit() {
  fl.ConfigFactoryDefault();
  fr.ConfigFactoryDefault();
  bl.ConfigFactoryDefault();
  br.ConfigFactoryDefault();
  std::thread visionThread(vision);
  visionThread.detach();
  gate.Set(0);
  
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  motorstart = fl.GetSelectedSensorPosition();
  startime = frc::Timer::GetFPGATimestamp();

}
void Robot::AutonomousPeriodic() {
  if((frc::Timer::GetFPGATimestamp())<= startime+5_s){
    conveyor.Set(ControlMode::PercentOutput,1);
    roller.Set(ControlMode::PercentOutput,1);
  }
  if((frc::Timer::GetFPGATimestamp())>= startime+5_s){
    conveyor.Set(ControlMode::PercentOutput,0);
    roller.Set(ControlMode::PercentOutput,0);
    if(fl.GetSelectedSensorPosition()>=(motorstart-(2048*7.31*1.8 ))){
      fl.Set(ControlMode::PercentOutput,-.3);
      fr.Set(ControlMode::PercentOutput,-.3);
      bl.Set(ControlMode::PercentOutput,-.3);
      br.Set(ControlMode::PercentOutput,-.3);

    }
    else{
      fl.Set(ControlMode::PercentOutput,0);
      fr.Set(ControlMode::PercentOutput,0);
      bl.Set(ControlMode::PercentOutput,0);
      br.Set(ControlMode::PercentOutput,0);
    }
    sl.Set(0);
    sr.Set(1);
  }
}

void Robot::TeleopInit() {
}
void Robot::TeleopPeriodic() {
  
//gate servo controls
  if(otherCon.GetRightBumper()==true){
    gate.Set(1);
  }
  if(otherCon.GetLeftBumper()==true){
    gate.Set(0);
  }
  yawcam.Set(.5+(-driveCon.GetLeftTriggerAxis()+ driveCon.GetRightTriggerAxis()));
  upcam.Set(driveCon.GetRightY()+.5);



//drive
  horz=-driveCon.GetRawAxis(0)*(driveCon.GetRawAxis(3)+.30);
  vert=driveCon.GetRawAxis(1)*(driveCon.GetRawAxis(3)+.25);
  pivot =-driveCon.GetRawAxis(4)*(driveCon.GetRawAxis(3)+.25);
  fl.Set(ControlMode::PercentOutput, (pivot+ vert+horz));
  fr.Set(ControlMode::PercentOutput, (-pivot+ vert-horz));
  bl.Set(ControlMode::PercentOutput, (pivot+ vert-horz));
  br.Set(ControlMode::PercentOutput, (-pivot+ vert+horz));

//climb
  climb1.Set(ControlMode::PercentOutput,(-otherCon.GetLeftTriggerAxis()+ otherCon.GetRightTriggerAxis()));
  climb2.Set(ControlMode::PercentOutput,(-otherCon.GetLeftTriggerAxis()+ otherCon.GetRightTriggerAxis()));
  climb3.Set(ControlMode::PercentOutput,(-otherCon.GetLeftTriggerAxis()+ otherCon.GetRightTriggerAxis()));
  climb4.Set(ControlMode::PercentOutput,(-otherCon.GetLeftTriggerAxis()+ otherCon.GetRightTriggerAxis()));

//intake
if((otherCon.GetYButton() || otherCon.GetAButton())==true){
  if(otherCon.GetYButton()){
    intake.Set(ControlMode::PercentOutput,.5);
  }
  if(otherCon.GetAButton()){
    intake.Set(ControlMode::PercentOutput,-.5);
  }
}
else{
  intake.Set(ControlMode::PercentOutput,0);
}

  
  
  
//conveyor

  conveyor.Set(ControlMode::PercentOutput,-otherCon.GetLeftY());
  
  if(otherCon.GetXButton()){
  roller.Set(ControlMode::PercentOutput,1);
  }
  else{
    roller.Set(ControlMode::PercentOutput,0);

  }
  
  sl.Set(0);
  
  sr.Set(1);
  
  
  
}


void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
}
void Robot::TestPeriodic() {
    
//gate servo controls
  if(otherCon.GetRightBumper()==true){
    gate.Set(1);
  }
  if(otherCon.GetLeftBumper()==true){
    gate.Set(0);
  }
  yawcam.Set(.5+(-driveCon.GetLeftTriggerAxis()+ driveCon.GetRightTriggerAxis()));
  upcam.Set(driveCon.GetRightY()+.5);



//drive
  horz=-driveCon.GetRawAxis(0)*(driveCon.GetRawAxis(3)+.30);
  vert=driveCon.GetRawAxis(1)*(driveCon.GetRawAxis(3)+.25);
  pivot =-driveCon.GetRawAxis(4)*(driveCon.GetRawAxis(3)+.25);
  //fl.Set(ControlMode::PercentOutput, (pivot+ vert+horz));
  //fr.Set(ControlMode::PercentOutput, (-pivot+ vert-horz));
  //bl.Set(ControlMode::PercentOutput, (pivot+ vert-horz));
  //br.Set(ControlMode::PercentOutput, (-pivot+ vert+horz));

//climb
  climb1.Set(ControlMode::PercentOutput,0.5*(-otherCon.GetLeftTriggerAxis()+ otherCon.GetRightTriggerAxis()));
  climb2.Set(ControlMode::PercentOutput,0.5*(-otherCon.GetLeftTriggerAxis()+ otherCon.GetRightTriggerAxis()));
  climb3.Set(ControlMode::PercentOutput,0.5*(-otherCon.GetLeftTriggerAxis()+ otherCon.GetRightTriggerAxis()));
  climb4.Set(ControlMode::PercentOutput,0.5*(-otherCon.GetLeftTriggerAxis()+ otherCon.GetRightTriggerAxis()));


  
//conveyor

  conveyor.Set(ControlMode::PercentOutput,.5*-otherCon.GetLeftY());
  
  if(otherCon.GetXButton()){
  roller.Set(ControlMode::PercentOutput,.5);
  }
  else{
    roller.Set(ControlMode::PercentOutput,0);

  }
  
  sl.Set(0);
  
  sr.Set(1);
  
  
  

}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
