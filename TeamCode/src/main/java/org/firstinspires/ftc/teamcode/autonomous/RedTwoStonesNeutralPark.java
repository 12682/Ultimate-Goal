package org.firstinspires.ftc.teamcode.autonomous;

import com.goldenratiorobotics.robot.body.components.SensorColor;
import com.goldenratiorobotics.robot.body.components.SensorRangeREV;
import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;
import com.goldenratiorobotics.robot.body.latcher.Latcher;
import com.goldenratiorobotics.robot.body.pincher.Pincher;
import com.goldenratiorobotics.robot.brain.gyro.Gyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Red 2 Stones Neutral Park", group="A. Autonomous")
//@Disabled
public class RedTwoStonesNeutralPark extends LinearOpMode {

    private ElapsedTime    runtime     = new ElapsedTime();
    private DriveTrain     driveTrain  = null;
    private Gyro           gyro        = null;
    private Latcher        latcher     = null;
    private Pincher        pincher     = null;
    private SensorRangeREV frontRange  = null;
    private SensorRangeREV backRange   = null;
    private SensorRangeREV rightRange  = null;
    private SensorRangeREV leftRange   = null;
    private SensorColor    bottomColor = null;

    private int    step     = 0;
    private int    maxSteps = 7;
    private double st;
    private double timeOut;

    @Override
    public void runOpMode() {
        driveTrain  = new DriveTrain(hardwareMap);
        gyro        = new Gyro(hardwareMap);
        latcher     = new Latcher(hardwareMap);
        pincher     = new Pincher(hardwareMap);
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

                telemetry.addData("Step", step + " / " + maxSteps);
                telemetry.update();
                step++;
            }

            //Moving to Stone 0
            if (step == 1) {
                //move forward to Stones
                st      = System.currentTimeMillis();
                timeOut = 6000;
                driveTrain.moveForward(.25);
                while ((frontRange.readCM() > 30) && (System.currentTimeMillis() - st < timeOut)) {
                    telemetry.addData("Front Range CM", frontRange.readCM());
                    telemetry.update();
                }
                driveTrain.reset();

                //Strafe to Stone
                st      = System.currentTimeMillis();
                timeOut = 3000;
                driveTrain.moveRight(.25);
                while ((leftRange.readCM() < 84) && (System.currentTimeMillis() - st < timeOut)) {
                    telemetry.addData("Left Range CM", leftRange.readCM());
                    telemetry.update();
                }
                driveTrain.reset();

                telemetry.addData("Step", step + " / " + maxSteps);
                telemetry.update();
                step++;
            }

            //Grab Stone 0
            if (step == 2) {
                //Lower Pivot
                pincher.lowerPivotAuto(.40, 2000);

                //Allow the motor to move freely
                pincher.setFloatMode();

                //Wiggle test
                driveTrain.reset();
                driveTrain.moveRightToPosition(.25, 50);
                driveTrain.reset();
                driveTrain.moveLeftToPosition(.25, 50);
                driveTrain.reset();

                //Pinch Stone
                pincher.pinch();
                sleep(500);

                //Set the motor back to brake mode
                pincher.setBrakeMode();

                //Bring Pivot Back
                pincher.backPivotAuto(.40, 2500);

                telemetry.addData("Step", step + " / " + maxSteps);
                telemetry.update();
                step++;
            }

            //Deliver Stone 0
            if (step == 3) {
                //Move past Skybridge
                driveTrain.moveRightToPosition(1, 2800);
                driveTrain.reset();

                //Lower pivot
                pincher.lowerPivotAuto(.40, 1000);

                //Release Stone
                pincher.release();
                sleep(500);

                //Move Pivot back
                pincher.backPivotAuto(.40, 2500);

                telemetry.addData("Step", step + " / " + maxSteps);
                telemetry.update();
                step++;
            }

            //Move to Stone 3
            if (step == 4) {
                //Strafe to the stone
                st      = System.currentTimeMillis();
                timeOut = 12000;
                driveTrain.moveLeft(.40);
                while ((leftRange.readCM() > 44) && (System.currentTimeMillis() - st < timeOut)) {
                    telemetry.addData("Left Range CM", leftRange.readCM());
                    telemetry.update();
                }
                driveTrain.reset();

                //Move forward to the stone
                st      = System.currentTimeMillis();
                timeOut = 6000;
                driveTrain.moveForward(.25);
                while ((frontRange.readCM() > 32) && (System.currentTimeMillis() - st < timeOut)) {
                    telemetry.addData("Front Range CM", frontRange.readCM());
                    telemetry.update();
                }
                driveTrain.reset();

                telemetry.addData("Step", step + " / " + maxSteps);
                telemetry.update();
                step++;
            }

            //Grab Stone 3
            if (step == 5) {
                //Lower Pivot
                pincher.lowerPivotAuto(.40, 2000);

                //Allow the motor to move freely
                pincher.setFloatMode();

                //Wiggle test
                driveTrain.reset();
                driveTrain.moveRightToPosition(.25, 100);
                driveTrain.reset();
                driveTrain.moveLeftToPosition(.25, 100);
                driveTrain.reset();

                //Pinch Stone
                pincher.pinch();
                sleep(500);

                //Set the motor back to brake mode
                pincher.setBrakeMode();

                //Bring Pivot Back
                pincher.backPivotAuto(.40, 2500);

                telemetry.addData("Step", step + " / " + maxSteps);
                telemetry.update();
                step++;
            }

            //Deliver Stone 3
            if (step == 6) {
                //Move closer to Neutral Park
                st      = System.currentTimeMillis();
                timeOut = 1500;
                driveTrain.moveForward(.50);
                while ((backRange.readCM() < 64) && (System.currentTimeMillis() - st < timeOut)) {
                    telemetry.addData("Back Range CM", backRange.readCM());
                    telemetry.update();
                }
                driveTrain.reset();

                //Move past Skybridge
                driveTrain.moveRightToPosition(1, 4800);
                driveTrain.reset();

                //Lower pivot
                pincher.lowerPivotAuto(.40, 1500);

                //Release Stone
                pincher.release();
                sleep(500);

                //Move Pivot back
                pincher.backPivotAuto(.40, 2500);

                telemetry.addData("Step", step + " / " + maxSteps);
                telemetry.update();
                step++;
            }

            //Park
            if (step == 7) {
                //Move closer to Neutral Park
                st      = System.currentTimeMillis();
                timeOut = 1500;
                driveTrain.moveForward(.50);
                while ((backRange.readCM() < 64) && (System.currentTimeMillis() - st < timeOut)) {
                    telemetry.addData("Back Range CM", backRange.readCM());
                    telemetry.update();
                }
                driveTrain.reset();

                //Park
                driveTrain.moveLeftToPosition(.50, 1000);
                driveTrain.reset();

                telemetry.addData("Step", step + " / " + maxSteps);
                telemetry.update();
                step++;
            }

            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            //endregion
        }
    }
}
