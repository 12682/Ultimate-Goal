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

@Autonomous(name="Blue Stone Foundation Neutral Park", group="A. Autonomous")
//@Disabled
public class BlueStoneFoundationNeutralPark extends LinearOpMode {

    private ElapsedTime    runtime     = new ElapsedTime();
    private DriveTrain     driveTrain  = null;
    private Gyro           gyro        = null;
    private Latcher        latcher     = null;
    private Pincher        pincher     = null;
    private Vision         vision      = null;
    private SensorRangeREV frontRange  = null;
    private SensorRangeREV backRange   = null;
    private SensorRangeREV rightRange  = null;
    private SensorRangeREV leftRange   = null;
    private SensorColor    bottomColor = null;

    private int       step     = 0;
    private int       maxSteps = 6;
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
                pincher.lowerPivotAuto(.45, 2000);

                //Allow the motor to move freely
                pincher.setFloatMode();

                //Wiggle test
                driveTrain.reset();
                driveTrain.moveLeftToPosition(.30, 100);
                driveTrain.reset();
                driveTrain.moveRightToPosition(.30, 100);
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
                pincher.backPivotAuto(.45, 2500);

                step++;
            }

            //Move to Foundation
            if (step == 3) {
                //Move up to avoid hitting other robot (Only an issue for Skystone 0 and somewhat for 1)
                if (skystone[0]) {
                    st = System.currentTimeMillis();
                    timeOut = 1000;
                    driveTrain.moveForward(.20);
                    while ((backRange.readCM() < 62) && (System.currentTimeMillis() - st < timeOut)) {
                    }
                    driveTrain.reset();
                } else if (skystone[1] || skystone[2]) {
                    st = System.currentTimeMillis();
                    timeOut = 1000;
                    driveTrain.moveForward(.20);
                    while ((backRange.readCM() < 59) && (System.currentTimeMillis() - st < timeOut)) {
                    }
                    driveTrain.reset();
                }

                //Strafe closer to the foundation wall
                driveTrain.moveLeftToPosition(.55, 2000);

                //Get closer to center of foundation
                st      = System.currentTimeMillis();
                timeOut = 4500;
                driveTrain.moveLeft(.55);
                while ((leftRange.readCM() > 50) && (System.currentTimeMillis() - st < timeOut)) {
                }
                driveTrain.reset();
                sleep(100);

                //Move to foundation
                st      = System.currentTimeMillis();
                timeOut = 6500;
                driveTrain.moveForward(.50);
                while ((backRange.readCM() < 75) && (System.currentTimeMillis() - st < timeOut)) {
                }
                driveTrain.reset();
                sleep(100);

                step++;
            }

            //Latch
            if (step == 4) {
                latcher.latch();
                sleep(500);

                step++;
            }

            //Reposition & Place
            if (step == 5) {
                //Reposition
                st      = System.currentTimeMillis();
                timeOut = 3500;
                driveTrain.moveBackward(.40);
                while ((backRange.readCM() > 7) && (System.currentTimeMillis() - st < timeOut)) {
                }
                driveTrain.reset();
                sleep(100);

                //Unlatch from foundation
                latcher.unlatch();

                //Place Stone on Foundation
                pincher.lowerPivotAuto(.45, 1200);
                pincher.release();
                sleep(100);

                //Pivot arm back
                pincher.backPivotAuto(.45, 2500);

                step++;
            }

            //Park
            if (step == 6) {
                //Move out of way of Foundation
                st      = System.currentTimeMillis();
                timeOut = 4000;
                driveTrain.moveRight(.55);
                while ((leftRange.readCM() < 100) && (System.currentTimeMillis() - st < timeOut)) {
                }
                driveTrain.reset();
                sleep(100);

                //Move closer to Neutral Park
                st      = System.currentTimeMillis();
                timeOut = 2000;
                driveTrain.moveForward(.35);
                while ((backRange.readCM() < 53.5) && (System.currentTimeMillis() - st < timeOut)) {
                }
                driveTrain.reset();
                sleep(100);

                //Bump the Foundation closer to the corner
                driveTrain.moveLeftToPosition(1,1000);

                //Park
                driveTrain.moveRightToPosition(.45, 1550);
                driveTrain.reset();
                sleep(100);

                step++;
            }
        }
    }
}
