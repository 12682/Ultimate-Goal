package org.firstinspires.ftc.teamcode.autonomous;

import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;
import com.goldenratiorobotics.robot.body.odometry.OdometryUnit;
import com.goldenratiorobotics.robot.brain.smart.SmartOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Auto Park", group = "A. Autonomous")
//@Disabled
public class AutoPark extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain driveTrain;
    private OdometryUnit odometryUnit;
    private SmartOdometry smartOdometry;
    int Stage = 0;

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
           if (Stage == 0) {
               telemetry.addData("Stage", Stage);
               telemetry.update();
               smartOdometry.moveBackward(DistanceUnit.CM,180,.3,.65,2800);
                Stage++;
            }
            if (Stage == 1) {
                telemetry.addData("Stage", Stage);
                telemetry.update();
                driveTrain.stop();
                Stage = 999;
            }

            //region telemetry
            telemetry.addData("Stage", Stage);
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
