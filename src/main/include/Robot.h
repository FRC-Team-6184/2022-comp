// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/Servo.h>
#include <frc/TimedRobot.h>
#include <ctre/Phoenix.h>
#include "frc/drive/MecanumDrive.h"
#include "frc/XboxController.h"
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <ctre/phoenix/music/Orchestra.h>
#include "cameraserver/CameraServer.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "frc/Timer.h"
#include "frc/GenericHID.h"
class Robot : public frc::TimedRobot {

 public:
  
  Orchestra orch;
  
  
  TalonFX fl{0};
  TalonFX fr{1};
  TalonFX bl{2};
  TalonFX br{3};
  frc::Servo sl{4};
  frc::Servo sr{5};

  
  VictorSPX climb1{4};
  VictorSPX climb2{5};
  VictorSPX climb3{6};
  VictorSPX climb4{7};

  TalonSRX intake{14};
  TalonSRX conveyor{15};
  TalonSRX roller{12};
  

  
  frc::Servo gate{1};

  frc::Servo yawcam{7};
  frc::Servo upcam{6};


  frc::XboxController driveCon{0};
  frc::XboxController otherCon{1};
  


 Robot(void){
  fl.Set(ControlMode::PercentOutput, 0);
  fr.Set(ControlMode::PercentOutput, 0);
  bl.Set(ControlMode::PercentOutput, 0);
  br.Set(ControlMode::PercentOutput, 0);
  fl.SetInverted(true);
  bl.SetInverted(true);
  climb1.SetInverted(true);
  climb2.SetInverted(true);
  

 } 
  
  
  
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;

  private:
    //vision
    static void vision(){
      cs::UsbCamera camFront = frc::CameraServer::StartAutomaticCapture();
      camFront.SetResolution(640, 480);
      cs::CvSink cvSink = frc::CameraServer::GetVideo();
      cs::CvSource outputStreamStd = frc::CameraServer::PutVideo("Gray", 640, 480);
      cv::Mat source;
      cv::Mat output;
      
      while(true) {
        if (cvSink.GrabFrame(source) == 0) {
          continue;
        }
        cvtColor(source, output, cv::COLOR_BGR2RGB);
        outputStreamStd.PutFrame(output);
      }

    };
};
