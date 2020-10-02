package org.firstinspires.ftc.teamcode.autonomous;

import com.goldenratiorobotics.robot.body.components.SensorColor;
import com.goldenratiorobotics.robot.body.components.SensorRangeREV;
import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;
import com.goldenratiorobotics.robot.body.latcher.Latcher;
import com.goldenratiorobotics.robot.body.pincher.Pincher;
import com.goldenratiorobotics.robot.brain.gyro.Gyro;
import com.goldenratiorobotics.robot.brain.vision.Vision;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Stone Neutral Park", group="A. Autonomous")
//@Disabled
public class BlueStoneNeutralPark extends LinearOpMode {

    private ElapsedTime    runtime     = new ElapsedTime();
    private DriveTrain     driveTrain  = null;
    private Gyro           gyro        = null;
    private Latcher        latcher     = null;
    private Pincher        pincher     = null;
    private Vision vision      = null;
    private SensorRangeREV frontRange  = null;
    private SensorRangeREV backRange   = null;
    private SensorRangeREV rightRange  = null;
    private SensorRangeREV leftRange   = null;
    private SensorColor    bottomColor = null;

    private int       step     = 0;
    private int       maxSteps = 4;
    private double    st;
    private double    timeOut;
    private int[]     stones   = new int[] {-1, -1, -1};
    private boolean[] skystone = new boolean[] {false, false, false};

    @Override
    public void runOpMode() {
        driveTrain  = new DriveTrain(hardwareMap);
        gyro        = new Gyro(hardwareMap);
        latcher     = new Latcher(hardwareMap);
        pincher     = new Pincher(hardwareMap);
        vision      = new Vision(hardwareMap);
        frontRange  = SensorRangeREV.getInstance(hardwareMap, "frontRange");
        backRange   = SensorRangeREV.getInstance(hardwareMap, "backRange");
        rightRange  = SensorRangeREV.getInstance(hardwareMap, "rightRange");
        leftRange   = SensorRangeREV.getInstance(hardwareMap, "leftRange");
        bottomColor = SensorColor.getInstance(hardwareMap, "bottomColor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //Starting
            if (step == 0) {
                latcher.unlatch();
                driveTrain.reset();

                //Use vision to detect the skystone
                stones = vision.returnValues();

                //Save which stone is the skystone
                //The way we set up the robot affects our logic:
                //If it sees the first stone (second left to right), etc.
                //Our robot is shifted one block to the right
                if (stones[0] == 0) {
                    skystone[1] = true;
                } else if (stones[1] == 0) {
                    skystone[2] = true;
                } else {
                    skystone[0] = true;
                }

                telemetry.addData("Left", skystone[0]);
                telemetry.addData("Mid", skystone[1]);
                telemetry.addData("Right", skystone[2]);
                telemetry.addData("Step", step + " / " + maxSteps);
                telemetry.update();
                step++;
            }

            //Moving to Stone
            if (step == 1) {
                //move to Stones
                st = System.currentTimeMillis();
                timeOut = 6000;
                driveTrain.moveForward(.20);
                while ((frontRange.readCM() > 29.5) && (System.currentTimeMillis() - st < timeOut)) {
                }
                driveTrain.reset();
                sleep(100);

                //Strafe to for SkyStone (0, 1, or 2)
                if (skystone[0]) {
                    //set up for Stone 0
                    st = System.currentTimeMillis();
                    timeOut = 4000;
                    driveTrain.moveLeft(.20);
                    while ((rightRange.readCM() < 83) && (System.currentTimeMillis() - st < timeOut)) {
                    }
                    driveTrain.reset();
                } else if (skystone[1]) {
                    //set up for Stone 1
                    st = System.currentTimeMillis();
                    timeOut = 4000;
                    driveTrain.moveLeft(.20);
                    while ((rightRange.readCM() < 68.5) && (System.currentTimeMillis() - st < timeOut)) {
                    }
                    driveTrain.reset();
                } else {
                    //Strafe to Stone 2
                    st      = System.currentTimeMillis();
                    timeOut = 400;
                    driveTrain.moveRight(.20);
                    while ((rightRange.readCM() > 57.5) && (System.currentTimeMillis() - st < timeOut)) {
                    }
                    driveTrain.reset();
                }
                sleep(100);

                step++;
            }

            //Grab Stone
            if (step == 2) {
                //Lower Pivot
                pincher.lowerPivotAuto(.40, 2000);

                //Allow the motor to move freely
                pincher.setFloatMode();

                //Wiggle test
                driveTrain.reset();
                driveTrain.moveRightToPosition(.30, 100);
                driveTrain.reset();
                driveTrain.moveLeftToPosition(.30, 100);
                driveTrain.reset();
                driveTrain.moveBackwardToPosition(.30, 100);
                driveTrain.reset();
                driveTrain.moveForwardToPosition(.30, 100);
                driveTrain.reset();

                //Pinch Stone
                pincher.pinch();
                sleep(500);

                //Set the motor back to brake mode
                pincher.setBrakeMode();

                //Bring Pivot Back
                pincher.backPivotAuto(.40, 2500);

                telemetry.addData("Left", skystone[0]);
                telemetry.addData("Mid", skystone[1]);
                telemetry.addData("Right", skystone[2]);
                telemetry.addData("Step", step + " / " + maxSteps);
                telemetry.update();
                step++;
            }

            //Deliver
            if (step == 3) {
                //Move past Skybridge
            if (skystone[0]) {
                driveTrain.moveLeftToPosition(.25, 2800);
                driveTrain.reset();
                sleep(500);
            }else if (skystone[1]){
                driveTrain.moveLeftToPosition(.25, 3500);
                driveTrain.reset();
                sleep(500);
            } else {driveTrain.moveLeftToPosition(.25, 4000);
                driveTrain.reset();
                sleep(500);
            }
            //Lower pivot
                pincher.lowerPivotAuto(.4, 2500);
                sleep(500);

                //Release Stone
                pincher.release();
                sleep(500);

                //Move Pivot back
                pincher.backPivotAuto(.4, 2500);
                sleep(500);

                telemetry.addData("Step", step + " / " + maxSteps);
                telemetry.update();
                step++;
            }

            //Park
            if (step == 4) {
                //Move closer to Neutral Park
                st      = System.currentTimeMillis();
                timeOut = 1500;
                driveTrain.moveForward(.25);
                while ((backRange.readCM() < 59) && (System.currentTimeMillis() - st < timeOut)) {
                    telemetry.addData("Back Range CM", backRange.readCM());
                    telemetry.update();
                }
                driveTrain.reset();
                sleep(500);

                //Park
                driveTrain.moveRightToPosition(.25, 1000);
                driveTrain.reset();
                sleep(500);

                telemetry.addData("Step", step + " / " + maxSteps);
                telemetry.update();
                step++;
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Step", step + " / " + maxSteps);
            telemetry.update();
        }
    }
}
