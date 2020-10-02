package org.firstinspires.ftc.teamcode.tests.systems.odometry;

import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;
import com.goldenratiorobotics.robot.body.odometry.OdometryUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test Odometry", group="C. Tests System")
//@Disabled
public class TestOdometry extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain driveTrain;
    private OdometryUnit odometryUnit;

    private double moveX;
    private double moveY;
    private double rotX;

    @Override
    public void runOpMode() {
        driveTrain = new DriveTrain(hardwareMap);
        odometryUnit = new OdometryUnit(hardwareMap, "leftPod", "rightPod", "horizontalPod");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        odometryUnit.startThread();

        while (opModeIsActive()) {
            moveX = gamepad1.left_stick_x;
            moveY = gamepad1.left_stick_y;
            rotX  = gamepad1.right_stick_x;
            driveTrain.moveTrig(moveX, -moveY, rotX);

            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Vertical Left Position", odometryUnit.getVerticalLeftPosition());
            telemetry.addData("Vertical Right Position", odometryUnit.getVerticalRightPosition());
            telemetry.addData("Horizontal Position", odometryUnit.getHorizontalPosition());
            telemetry.addData("(X, Y, Theta)", "(" + odometryUnit.getXYTheta()[0] + ", " + odometryUnit.getXYTheta()[1] + ", " + odometryUnit.getXYTheta()[2] + ")");
            telemetry.addData("(X, Y) IN", "(" + odometryUnit.getXIN() + ", " + odometryUnit.getYIN() + ")");
            telemetry.addData("(X, Y) CM", "(" + odometryUnit.getXCM() + ", " + odometryUnit.getYCM() + ")");
            telemetry.addData("Thread is Active", odometryUnit.isThreadActive());
            telemetry.update();
            //endregion
        }

        odometryUnit.stopThread();
    }
}
