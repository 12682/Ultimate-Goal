package org.firstinspires.ftc.teamcode.tests.systems.odometry;

import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;
import com.goldenratiorobotics.robot.body.odometry.OdometryUnit;
import com.goldenratiorobotics.robot.brain.smart.SmartMath;
import com.goldenratiorobotics.robot.brain.smart.SmartOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Point;

@TeleOp(name="Test OdometryForward", group="C. Tests System")
//@Disabled
public class TestOdometryForward extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain driveTrain;
    private OdometryUnit odometryUnit;
    private boolean running = true;

    double st;
    double timeOut;
    double distance;
    double maxSpeed;
    double minSpeed;
    double orientation;
    double heading;
    Point startPos;
    Point currentPos;
    Point endPos;
    double distanceTravled;
    double goalDistance;
    double speed;

    @Override
    public void runOpMode() {
        driveTrain = new DriveTrain(hardwareMap);
        odometryUnit = new OdometryUnit(hardwareMap, "rightBack", "leftFront", "rightFront");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        odometryUnit.start();

        while (opModeIsActive()) {

           if (running){
               DistanceUnit distanceUnit = DistanceUnit.CM;
               st = System.currentTimeMillis();
               timeOut = 4000;
               distance = 80;
               maxSpeed = .4;
               minSpeed = .1;
               orientation = odometryUnit.returnOrientation();
               heading = orientation - 90;
               startPos = odometryUnit.returnPointUnits(distanceUnit).clone();
               currentPos = startPos.clone();
               endPos = new Point(startPos.x + (distance * SmartMath.cos(heading)), startPos.y - (distance * SmartMath.sin(heading)));
               distanceTravled = 0;
               goalDistance = SmartMath.distanceFormula(endPos,startPos);
               speed = maxSpeed;

               driveTrain.moveForward(speed);

               while ((distanceTravled < goalDistance) && (System.currentTimeMillis() - st < timeOut)){
                   orientation = odometryUnit.returnOrientation();
                   heading = orientation - 90;
                   currentPos = odometryUnit.returnPointUnits(distanceUnit).clone();
                   distanceTravled = SmartMath.distanceFormula(currentPos,startPos);
                   speed = maxSpeed + ((minSpeed - maxSpeed) * (distanceTravled / goalDistance));
                   driveTrain.moveForward(speed);
               }
               driveTrain.stop();
           }
           running = false;

            //region telemetry
            telemetry.addData("Orientation", orientation);
            telemetry.addData("Heading", heading);
            telemetry.addData("Start Pos", startPos.toString());
            telemetry.addData("Current Pos", currentPos.toString());
            telemetry.addData("End Pos", endPos.toString());
            telemetry.addData("Distance Traveled", distanceTravled);
            telemetry.addData("Goal Distance", goalDistance);
            telemetry.addData("Speed", speed);
            telemetry.update();
            //endregion
        }

        odometryUnit.stop();
    }
}
