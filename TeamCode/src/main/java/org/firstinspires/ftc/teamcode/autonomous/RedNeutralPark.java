package org.firstinspires.ftc.teamcode.autonomous;

import com.goldenratiorobotics.robot.body.components.SensorColor;
import com.goldenratiorobotics.robot.body.components.SensorRangeREV;
import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;
import com.goldenratiorobotics.robot.body.latcher.Latcher;
import com.goldenratiorobotics.robot.brain.gyro.Gyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Red Neutral Park", group="A. Autonomous")
//@Disabled
public class RedNeutralPark extends LinearOpMode {

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

                telemetry.addData("Step", step);
                telemetry.update();
                step++;
            }
            sleep(500);

            //Moving to park
            if (step == 1) {
                st      = System.currentTimeMillis();
                timeOut = 5000;
                driveTrain.moveForward(.25);
                while ((backRange.readCM() < 55) && (System.currentTimeMillis() - st < timeOut)) {
                    telemetry.addData("Back Range CM", backRange.readCM());
                    telemetry.update();
                }
                driveTrain.reset();

                telemetry.addData("Step", step);
                telemetry.update();
                step++;
            }
            sleep(500);

            //park
            if (step == 2) {
                driveTrain.moveRightToPosition(.25, 800);
                driveTrain.reset();

                telemetry.addData("Step", step);
                telemetry.update();
                step++;
            }
            sleep(500);


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Step", step + " / " + 3);
            telemetry.update();
        }
    }
}
