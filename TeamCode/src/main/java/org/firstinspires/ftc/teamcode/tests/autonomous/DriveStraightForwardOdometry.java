package org.firstinspires.ftc.teamcode.tests.autonomous;

import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;
import com.goldenratiorobotics.robot.body.odometry.OdometryUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Drive Straight Forward Odometry", group="B. Builder Tests")
//@Disabled
public class DriveStraightForwardOdometry extends LinearOpMode {

    private ElapsedTime runtime    = new ElapsedTime();
    private DriveTrain driveTrain;
    private OdometryUnit odometryUnit;

    private double  speed      = .25;
    private double distance = 50;
    private boolean done       = false;


    @Override
    public void runOpMode() {
        driveTrain = new DriveTrain(hardwareMap);
        odometryUnit = new OdometryUnit(hardwareMap, "leftPod", "rightPod", "horizontalPod");

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Speed", speed);
        telemetry.addData("Distance", distance + " in");
        telemetry.update();

        waitForStart();
        runtime.reset();

        odometryUnit.startThread();

        while (opModeIsActive()) {

            while (!done) {
                driveTrain.moveForward(speed);
                while (odometryUnit.getYIN() < distance) {
                    telemetry.addData("Y position in", odometryUnit.getYIN() + " in");
                    telemetry.update();
                }
                driveTrain.stop();

                done = true;
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Done", done);
            telemetry.update();
        }

        odometryUnit.stopThread();
    }
}
