package org.firstinspires.ftc.teamcode.tests.autonomous;

import com.goldenratiorobotics.robot.body.components.SensorColor;
import com.goldenratiorobotics.robot.body.components.SensorRangeREV;
import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;
import com.goldenratiorobotics.robot.body.latcher.Latcher;
import com.goldenratiorobotics.robot.body.pincher.Pincher;
import com.goldenratiorobotics.robot.brain.gyro.Gyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="MVP Testing (Blue)", group="AB. Autonomous Tests")
//@Disabled
public class MVP extends LinearOpMode {

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

    private int    step = 0;
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

                telemetry.addData("Step", step + " / 6");
                telemetry.update();
                step++;
            }

            //Moving to Stone
            if (step == 1) {
                //move forward to Stones
                st      = System.currentTimeMillis();
                timeOut = 6000;
                driveTrain.moveForward(.20);
                while ((frontRange.readCM() > 28) && (System.currentTimeMillis() - st < timeOut)) {
                    telemetry.addData("Front Range CM", frontRange.readCM());
                    telemetry.update();
                }
                driveTrain.reset();
                sleep(100);

                //Strafe to Stone
                st      = System.currentTimeMillis();
                timeOut = 3000;
                driveTrain.moveLeft(.20);
                while ((rightRange.readCM() < 84) && (System.currentTimeMillis() - st < timeOut)) {
                    telemetry.addData("Right Range CM", leftRange.readCM());
                    telemetry.update();
                }
                driveTrain.reset();
                sleep(100);

                telemetry.addData("Step", step + " / 6");
                telemetry.update();
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
                driveTrain.moveLeftToPosition(.25, 100);
                driveTrain.reset();
                driveTrain.moveRightToPosition(.25, 100);
                driveTrain.reset();

                //Pinch Stone
                pincher.pinch();
                sleep(500);

                //Set the motor back to brake mode
                pincher.setBrakeMode();

                //Bring Pivot Back
                pincher.backPivotAuto(.40, 2500);

                telemetry.addData("Step", step + " / 6");
                telemetry.update();
                step++;
            }

            //Move to Foundation
            if (step == 3) {
                //Get closer to center of foundation
                st      = System.currentTimeMillis();
                timeOut = 9000;
                driveTrain.moveLeft(.45);
                while ((leftRange.readCM() > 55) && (System.currentTimeMillis() - st < timeOut)) {
                    telemetry.addData("Left Range CM", leftRange.readCM());
                    telemetry.update();
                }
                driveTrain.reset();
                sleep(100);

                //Move to foundation
                st      = System.currentTimeMillis();
                timeOut = 6000;
                driveTrain.moveForward(.45);
                while ((backRange.readCM() < 79) && (System.currentTimeMillis() - st < timeOut)) {
                    telemetry.addData("Back Range CM", backRange.readCM());
                    telemetry.update();
                }
                driveTrain.reset();
                sleep(100);

                telemetry.addData("Step", step + " / 6");
                telemetry.update();
                step++;
            }

            //Latch
            if (step == 4) {
                latcher.latch();
                sleep(500);

                telemetry.addData("Step", step + " / 6");
                telemetry.update();
                step++;
            }

            //Reposition & Place
            if (step == 5) {
                //Reposition
                st      = System.currentTimeMillis();
                timeOut = 6000;
                driveTrain.moveBackward(.45);
                while ((backRange.readCM() > 7) && (System.currentTimeMillis() - st < timeOut)) {
                    telemetry.addData("Back Range CM", backRange.readCM());
                    telemetry.update();
                }
                driveTrain.reset();
                sleep(100);

                //Unlatch from foundation
                latcher.unlatch();

                //Place Stone on Foundation
                pincher.lowerPivotAuto(.40, 1500);
                pincher.release();
                sleep(100);

                //Pivot arm back
                pincher.backPivotAuto(.40, 2500);

                telemetry.addData("Step", step + " / 6");
                telemetry.update();
                step++;
            }

            //Park
            if (step == 6) {
                //Move out of way of Foundation
                st      = System.currentTimeMillis();
                timeOut = 4000;
                driveTrain.moveRight(.35);
                while ((leftRange.readCM() < 95) && (System.currentTimeMillis() - st < timeOut)) {
                    telemetry.addData("Left Range CM", leftRange.readCM());
                    telemetry.update();
                }
                driveTrain.reset();
                sleep(100);

                //Move closer to Neutral Park
                st      = System.currentTimeMillis();
                timeOut = 2000;
                driveTrain.moveForward(.30);
                while ((backRange.readCM() < 65) && (System.currentTimeMillis() - st < timeOut)) {
                    telemetry.addData("Back Range CM", backRange.readCM());
                    telemetry.update();
                }
                driveTrain.reset();
                sleep(100);

                //Park
                driveTrain.moveRightToPosition(.35, 1250);
                driveTrain.reset();
                sleep(100);

                telemetry.addData("Step", step + " / 6");
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
