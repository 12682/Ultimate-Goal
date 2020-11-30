package com.goldenratiorobotics.robot.body.shooter;

import com.goldenratiorobotics.robot.body.components.Motor;
import com.goldenratiorobotics.robot.body.components.ServoNonContinuous;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private Motor shooterMotor = null;
    private ServoNonContinuous flipper = null;
    private double inPosition = 0.92;
    private double neutralPosition = 0.67;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = Motor.getInstance(hardwareMap,"shooterMotor");
        flipper      = ServoNonContinuous.getInstance(hardwareMap,"flipper");
        shooterMotor.reverseDirection();

    }
    public void runShooter(double speed) {
        shooterMotor.setSpeed(speed);
    }
    public void runShooterPosition(double speed, int position){
        shooterMotor.runToPosition(speed,position);
    }
    public double getSpeed(){
        return shooterMotor.getSpeed();
    }
    public int getMotorPosition(){
        return shooterMotor.getPosition();
    }
  //flipper goes from 0 - 1
    public void moveFlipper(double position) {
        flipper.setPosition(position);
    }
    //need to test values
    public void flipIn(){
        moveFlipper(inPosition);
    }
    public void neuterFlipper(){
        moveFlipper(neutralPosition);
    }
    public double getFlipperPosition(){
        return flipper.getPosition();
    }
    public boolean isFlipped(){
        return getFlipperPosition()>=inPosition;
    }

}
