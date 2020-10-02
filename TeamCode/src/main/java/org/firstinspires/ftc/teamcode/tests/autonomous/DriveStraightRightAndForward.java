package org.firstinspires.ftc.teamcode.tests.autonomous;

import com.goldenratiorobotics.robot.body.components.Motor;
import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Drive Right and Forward Straight", group="B. Builder Tests")
//@Disabled
public class DriveStraightRightAndForward extends LinearOpMode {

    private ElapsedTime runtime   = new ElapsedTime();
    private DriveTrain driveTrain = null;

    private double speed   = .25;
    private double st      = 0;
    private double timeOut = 2000;
    private double step    = 0;

    @Override
    public void runOpMode() {
        driveTrain = new DriveTrain(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Speed", speed);
        telemetry.addData("Time out", timeOut);
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (step == 0) {
                st = System.currentTimeMillis();
                driveTrain.moveRight(speed);
                while (System.currentTimeMillis() - st < timeOut) {
                    telemetry.addData("Time", System.currentTimeMillis() - st);
                    telemetry.update();
                }
                driveTrain.stop();
                sleep(500);

                telemetry.addData("Step", step);
                telemetry.update();
                step++;
            }

            if (step == 1) {
                st = System.currentTimeMillis();
                driveTrain.moveForward(speed);
                while (System.currentTimeMillis() - st < timeOut) {
                    telemetry.addData("Time", System.currentTimeMillis() - st);
                    telemetry.update();
                }
                driveTrain.stop();
                sleep(500);

                telemetry.addData("Step", step);
                telemetry.update();
                step++;
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Step", step);
            telemetry.update();
        }
    }
}
