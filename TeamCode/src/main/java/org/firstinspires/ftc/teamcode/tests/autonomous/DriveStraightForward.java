package org.firstinspires.ftc.teamcode.tests.autonomous;

import com.goldenratiorobotics.robot.body.components.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Drive Straight Forward", group="B. Builder Tests")
//@Disabled
public class DriveStraightForward extends LinearOpMode {

    private ElapsedTime runtime    = new ElapsedTime();
    private Motor rightFront = null;
    private Motor rightBack = null;
    private Motor leftFront = null;
    private Motor leftBack = null;

    private double  speed      = .25;
    private int     encoderMax = 5000;
    private boolean done       = false;


    @Override
    public void runOpMode() {
        rightFront = Motor.getInstance(hardwareMap, "rightFront");
        rightBack = Motor.getInstance(hardwareMap, "rightBack");
        leftFront = Motor.getInstance(hardwareMap, "leftFront");
        leftBack = Motor.getInstance(hardwareMap, "leftBack");

        rightFront.reverseDirection();
        rightBack.reverseDirection();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Speed", speed);
        telemetry.addData("Encoder Max", encoderMax);
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            while (!done) {
                leftFront.runToPosition(speed, encoderMax);
                rightFront.runToPosition(speed, encoderMax);
                leftBack.runToPosition(speed, encoderMax);
                rightBack.runToPosition(speed, encoderMax);
                while (rightFront.isBusy() && rightBack.isBusy() && leftBack.isBusy() && leftFront.isBusy()) {
                    //wait
                    telemetry.addData("Status", "Run Time: " + runtime.toString());
                    telemetry.addData("Encoder Value (RF)", rightFront.getPosition());
                    telemetry.addData("Encoder Value (RB)", rightBack.getPosition());
                    telemetry.addData("Encoder Value (LF)", leftFront.getPosition());
                    telemetry.addData("Encoder Value (LB)", leftBack.getPosition());
                    telemetry.addData("Done", done);
                    telemetry.update();
                }
                done = true;
            }

            leftFront.setSpeed(0);
            rightFront.setSpeed(0);
            leftBack.setSpeed(0);
            rightBack.setSpeed(0);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Done", done);
            telemetry.update();
        }
    }
}
