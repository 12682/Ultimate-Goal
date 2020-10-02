package org.firstinspires.ftc.teamcode.autonomous;

import com.goldenratiorobotics.robot.body.components.SensorColor;
import com.goldenratiorobotics.robot.body.components.SensorRangeREV;
import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;
import com.goldenratiorobotics.robot.body.latcher.Latcher;
import com.goldenratiorobotics.robot.brain.gyro.Gyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Foundation Wall Park", group="A. Autonomous")
//@Disabled
public class BlueFoundationWallPark extends LinearOpMode {

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
            sleep(500);

            //Moving to Foundation
            if (step == 1) {
                //get closer to center of foundation
                st      = System.currentTimeMillis();
                timeOut = 3000;
                driveTrain.moveLeft(.25);
                while ((leftRange.readCM() > 50) && (System.currentTimeMillis() - st < timeOut)) {
                    telemetry.addData("Right Range CM", leftRange.readCM());
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
            sleep(500);

            //Latch
            if (step == 2) {
                latcher.latch();
                sleep(500);

                telemetry.addData("Step", step);
                telemetry.update();
                step++;
            }
            sleep(500);

            //Move Foundation
            if (step == 3) {
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
            sleep(500);

            //Park closest to wall side
            if (step == 4) {
                //move out of way of foundation
                st      = System.currentTimeMillis();
                timeOut = 4000;
                driveTrain.moveRight(.25);
                while ((leftRange.readCM() < 110) && (System.currentTimeMillis() - st < timeOut)) {
                    telemetry.addData("Right Range CM", rightRange.readCM());
                    telemetry.update();
                }
                driveTrain.reset();
                sleep(500);

                //park
                driveTrain.moveRightToPosition(.25, 1200);

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
