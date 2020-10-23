package org.firstinspires.ftc.teamcode.tests.systems.odometry;

import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;
import com.goldenratiorobotics.robot.body.odometry.OdometryUnit;
import com.goldenratiorobotics.robot.brain.smart.SmartOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Test OdometryForward", group="C. Tests System")
//@Disabled
public class TestOdometryForward extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain driveTrain;
    private OdometryUnit odometryUnit;
    private SmartOdometry smartOdometry;

    @Override
    public void runOpMode() {
        driveTrain = new DriveTrain(hardwareMap);
        odometryUnit = new OdometryUnit(hardwareMap, "rightBack", "leftFront", "rightFront");
        smartOdometry = new SmartOdometry(driveTrain,odometryUnit);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        odometryUnit.start();

        while (opModeIsActive()) {
            smartOdometry.moveForward(DistanceUnit.CM,25,.3,.65,5000);

            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Vertical Left Position", odometryUnit.returnVL());
            telemetry.addData("Vertical Right Position", odometryUnit.returnVR());
            telemetry.addData("Horizontal Position", odometryUnit.returnH());
            telemetry.addData("(X, Y, Theta)", "(" + odometryUnit.returnPoint().x + ", " + odometryUnit.returnPoint().y + ", " + odometryUnit.returnOrientation() + ")");
            telemetry.addData("(X, Y) IN", "(" + odometryUnit.returnPointUnits(DistanceUnit.INCH).x + ", " + odometryUnit.returnPointUnits(DistanceUnit.INCH).y + ")");
            telemetry.addData("(X, Y) CM", "(" + odometryUnit.returnPointUnits(DistanceUnit.CM).x + ", " + odometryUnit.returnPointUnits(DistanceUnit.CM).y + ")");
            telemetry.addData("Thread is Active", odometryUnit.isAlive());
            telemetry.update();
            //endregion
        }

        odometryUnit.stop();
    }
}
