package org.firstinspires.ftc.teamcode.autonomous;

import com.goldenratiorobotics.robot.body.components.SensorColor;
import com.goldenratiorobotics.robot.body.components.SensorRangeREV;
import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;
import com.goldenratiorobotics.robot.body.latcher.Latcher;
import com.goldenratiorobotics.robot.brain.gyro.Gyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Red Foundation Wall Park", group="A. Autonomous")
//@Disabled
public class RedFoundationWallPark extends LinearOpMode {

    private ElapsedTime    runtime     = new ElapsedTime();
    private DriveTrain     driveTrain  = null;
    private Gyro           gyro        = null;
    private Latcher        latcher     = null;
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
                sleep(500);

                telemetry.addData("Step", step);
                telemetry.update();
                step++;
            }

            //Moving to Foundation
            if (step == 1) {
                //get closer to center of foundation
                st      = System.currentTimeMillis();
                timeOut = 3000;
                driveTrain.moveRight(.25);
                while ((rightRange.readCM() > 50) && (System.currentTimeMillis() - st < timeOut)) {
                    telemetry.addData("Right Range CM", rightRange.readCM());
                    telemetry.update();
                }
                driveTrain.reset();
                sleep(500);

                //move to foundation
                st      = System.currentTimeMillis();
                timeOut = 6000;
                driveTrain.moveForward(.25);
                while ((backRange.readCM() < 67) && (System.currentTimeMillis() - st < timeOut)) {
                    telemetry.addData("Back Range CM", backRange.readCM());
                    telemetry.update();
                }
                driveTrain.reset();
                sleep(500);

                telemetry.addData("Step", step);
                telemetry.update();
                step++;
            }

            //Latch
            if (step == 2) {
                latcher.latch();
                sleep(500);

                telemetry.addData("Step", step);
                telemetry.update();
                step++;
            }

            //Move Foundation
            if (step == 3) {
                //WIGGLE/ANGLING METHOD, NOT POSSIBLE / NOT WORKING
                //Angle it to corner
//                st      = System.currentTimeMillis();
//                timeOut = 3000;
//                driveTrain.manualMoveAll(-.1, -.9,-.1, -.9);
//                while ((gyro.composeAngleYaw() > -30) && (System.currentTimeMillis() - st < timeOut)) {
//                    telemetry.addData("Yaw", gyro.composeAngleYaw());
//                    telemetry.update();
//                }
//                driveTrain.reset();
//                sleep(500);
//
//                //Push it forward to reposition
//                st      = System.currentTimeMillis();
//                timeOut = 3000;
//                driveTrain.manualMoveAll(.8, .3, .8, .3);
//                while ((gyro.composeAngleYaw() > -80) && (System.currentTimeMillis() - st < timeOut)) {
//                    telemetry.addData("Yaw", gyro.composeAngleYaw());
//                    telemetry.update();
//                }
//                driveTrain.reset();
//                sleep(500);

                //180 TURN METHOD, TOO MUCH TIME
//                driveTrain.moveLeftToPosition(.25, 600);
//                driveTrain.reset();
//                sleep(500);
//
//                //rotate it to corner
//                st      = System.currentTimeMillis();
//                timeOut = 8000;
//                driveTrain.rotateRight(.25);
//                while ((gyro.composeAngleYaw() > -150) && (System.currentTimeMillis() - st < timeOut)) {
//                    telemetry.addData("Yaw", gyro.composeAngleYaw());
//                    telemetry.update();
//                }
//                driveTrain.reset();
//                sleep(500);
//
//                //move it to corner
//                driveTrain.moveForwardToPosition(.25, 4000);
//                driveTrain.reset();
//                sleep(500);

                //Straight back method
                st      = System.currentTimeMillis();
                timeOut = 6000;
                driveTrain.moveBackward(.25);
                while ((backRange.readCM() > 1) && (System.currentTimeMillis() - st < timeOut)) {
                    telemetry.addData("Back Range CM", backRange.readCM());
                    telemetry.update();
                }
                driveTrain.reset();
                sleep(500);

                latcher.unlatch();
                sleep(500);

                telemetry.addData("Step", step);
                telemetry.update();
                step++;
            }

            //Park closest to wall side
            if (step == 4) {
                //WIGGLE/ANGLING METHOD, NOT WORKING
                //position closest to neutral zone
//                driveTrain.moveLeftToPosition(.25, 800);
//                driveTrain.reset();
//                sleep(500);

                //180 TURN METHOD, TOO MUCH TIME
                //position for park
//                driveTrain.moveBackwardToPosition(.25, 200);
//                driveTrain.reset();
//                sleep(500);
//
                //park
//                driveTrain.moveRightToPosition(.25, 2500);
//                driveTrain.reset();
//                sleep(500);

                //WIGGLE/ANGLING METHOD, NOT WORKING
                //park
//                st      = System.currentTimeMillis();
//                timeOut = 3500;
//                driveTrain.moveBackwardToPosition(.25, 1980);
//                driveTrain.reset();
//                sleep(500);

                //move out of way of foundation
                st      = System.currentTimeMillis();
                timeOut = 4000;
                driveTrain.moveLeft(.25);
                while ((rightRange.readCM() < 110) && (System.currentTimeMillis() - st < timeOut)) {
                    telemetry.addData("Right Range CM", rightRange.readCM());
                    telemetry.update();
                }
                driveTrain.reset();
                sleep(500);

                //park
                driveTrain.moveLeftToPosition(.25, 1200);

                telemetry.addData("Step", step);
                telemetry.update();
                step++;
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Step", step + " / " + 5);
            telemetry.update();
        }
    }
}
