package com.goldenratiorobotics.robot.body.wobbleGrabber;

import com.goldenratiorobotics.robot.body.components.LimitSwitchREV;
import com.goldenratiorobotics.robot.body.components.Motor;
import com.goldenratiorobotics.robot.body.components.ServoNonContinuous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class WobbleGrabber {
    private Motor arm;
    private ServoNonContinuous pincher;
    private LimitSwitchREV limitOut, limitIn;

    private double openPosition = .5;
    private double closedPosition= 0;


    public WobbleGrabber(HardwareMap hardwareMap) {
        arm = Motor.getInstance(hardwareMap, "arm");
        pincher = ServoNonContinuous.getInstance(hardwareMap, "pincher");
        limitOut = LimitSwitchREV.getInstance(hardwareMap, "limitOut");
        limitIn = LimitSwitchREV.getInstance(hardwareMap,"limitIn");

        arm.zeroPowerBrake();
    }

    public boolean isArmOut() {
        return limitOut.isPressed();

    }
    public boolean isArmIn(){
        return limitIn.isPressed();
    }

    public void pinchToPosition(double position) {
        pincher.setPosition(position);
    }
    public void pinch() {
        pinchToPosition(closedPosition);
    }
    public void release(){
        pinchToPosition(openPosition);
    }
    public double getPincherPosition(){
        return pincher.getPosition();
    }
    public boolean isPinched() {
        return getPincherPosition() == closedPosition;

    }

    //towards robot for positive, away from robot negative.
    public void runArmManual (double speed){
        arm.setSpeed(speed);
    }
    public void runArm (double speed) {
        if (isArmIn()){
            runArmManual(Range.clip(speed,-1,0));
        } else if (isArmOut()){
            runArmManual(Range.clip(speed,0,1));
        } else {
            runArmManual(speed);
        }
    }
    public void runArmToPosition (double speed, int position){
        arm.runToPosition(speed,position);
    }
    public double getArmSpeed(){
        return arm.getSpeed();
    }
    public int getArmPosition(){
        return arm.getPosition();
    }

}
