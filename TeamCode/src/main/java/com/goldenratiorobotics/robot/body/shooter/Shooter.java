package com.goldenratiorobotics.robot.body.shooter;

import com.goldenratiorobotics.robot.body.components.Motor;
import com.goldenratiorobotics.robot.body.components.ServoNonContinuous;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private Motor shooterMotor = null;
    private ServoNonContinuous flipper = null;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = Motor.getInstance(hardwareMap,"shooterMotor");
        flipper      = ServoNonContinuous.getInstance(hardwareMap,"flipper");

    }
    public void shootOut(double speed) {
        shooterMotor.setSpeed(speed);
    }

    public double getSpeed(){
        return shooterMotor.getSpeed();
    }
  //flipper goes from 0 - 1
    public void moveFlipper(double position) {
        flipper.setPosition(position);
    }

}
