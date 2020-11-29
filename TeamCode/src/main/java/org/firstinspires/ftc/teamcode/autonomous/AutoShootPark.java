package org.firstinspires.ftc.teamcode.autonomous;

import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;
import com.goldenratiorobotics.robot.body.odometry.OdometryUnit;
import com.goldenratiorobotics.robot.brain.smart.SmartOdometry;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.goldenratiorobotics.robot.body.shooter.Shooter;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Auto Shoot Park", group = "A. Autonomous")
//@Disabled
public class AutoShootPark extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain driveTrain;
    private OdometryUnit odometryUnit;
    private SmartOdometry smartOdometry;
    private Shooter        shooter     = null;
    private double shooterSpeed        =.8;
    int Stage = 0;

    @Override
    public void runOpMode() {
        driveTrain = new DriveTrain(hardwareMap);
        shooter     = new Shooter(hardwareMap);
        odometryUnit = new OdometryUnit(hardwareMap, "rightBack", "leftFront", "rightFront");
        smartOdometry = new SmartOdometry(driveTrain,odometryUnit);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        odometryUnit.start();

        while (opModeIsActive()) {
            // Drive to wall
           if (Stage == 0) {
                smartOdometry.moveBackward(DistanceUnit.CM,225,.3,.65,4800);
                driveTrain.stop();
                Stage++;
            }
           //shoot rings
            if (Stage == 1) {
                smartOdometry.rotateLeft(5,.1,.3,1000);
                driveTrain.stop();
                shooter.runShooter(.8);
                sleep(1500);
                shooter.flipIn();
                sleep(1500);
                shooter.neuterFlipper();
                sleep(1500);
                shooter.flipIn();
                sleep(1000);
                shooter.neuterFlipper();
                sleep(1000);
                shooter.flipIn();
                sleep(1000);
                shooter.neuterFlipper();
                sleep(1000);
                shooter.flipIn();
                sleep(1000);
                shooter.neuterFlipper();
                sleep(1000);
                shooter.flipIn();
                sleep(1000);
                shooter.neuterFlipper();
                sleep(1000);
                shooter.runShooter(0);
                smartOdometry.rotateRight(5,.1,.3,1000);
                driveTrain.stop();
                Stage++;
            }
            // Drive to park
            if (Stage == 2) {
                smartOdometry.moveForward(DistanceUnit.CM,159,.3,.65,2000);
                driveTrain.stop();
                Stage++;
            }
            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Vertical Left Position", odometryUnit.returnVL());
            telemetry.addData("Vertical Right Position", odometryUnit.returnVR());
            telemetry.addData("Horizontal Position", odometryUnit.returnH());
            telemetry.addData("(X, Y, Theta)", odometryUnit.returnPoint().toString());
            telemetry.addData("(X, Y) IN", odometryUnit.returnPointUnits(DistanceUnit.INCH).toString());
            telemetry.addData("(X, Y) CM", odometryUnit.returnPointUnits(DistanceUnit.CM).toString());
            telemetry.addData("Thread is Active", odometryUnit.isAlive());
            telemetry.update();
            //endregion
        }

        odometryUnit.stop();
    }
}
