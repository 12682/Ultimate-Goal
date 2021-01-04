package org.firstinspires.ftc.teamcode.tests.systems.odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@TeleOp(name="Odometry System Callibration", group="B. Calibration")
public class OdometryCalibration extends LinearOpMode {
    // Motors of the driveTrain
    DcMotor leftFront, rightFront, rightBack, leftBack;
    // Odometry pods
    DcMotor leftPod, rightPod, horizontalPod;

    // REV hub IMU
    BNO055IMU imu;

    // HardwareMap names of motors and odometry pods
    String lfName = "leftFront", rfName = "rightFront", rbName = "rightBack", lbName = "leftBack";
    String leftPodName = "rightBack", rightPodName = "leftFront", horizontalPodName = "rightFront";

    // Speed to rotate for calibration
    final double PIVOT_SPEED = 0.5;

    // Encoder ticks for every inch the robot moves
    final double COUNTS_PER_INCH = 307.699557;

    ElapsedTime timer = new ElapsedTime();

    double horizontalTickOffset = 0;

    // Text files in controller phone under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware map values
        rightFront = hardwareMap.dcMotor.get(rfName);
        rightBack  = hardwareMap.dcMotor.get(rbName);
        leftFront  = hardwareMap.dcMotor.get(lfName);
        leftBack   = hardwareMap.dcMotor.get(lbName);

        leftPod       = hardwareMap.dcMotor.get(leftPodName);
        rightPod      = hardwareMap.dcMotor.get(rightPodName);
        horizontalPod = hardwareMap.dcMotor.get(horizontalPodName);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware map Init complete");
        telemetry.update();

        // Initialize imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Initialize imu parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.clear();

        //Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();

        waitForStart();

        // Begin the calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code)
        while (getZAngle() < 90 && opModeIsActive()) {
            rightFront.setPower(-PIVOT_SPEED);
            rightBack.setPower(-PIVOT_SPEED);
            leftFront.setPower(PIVOT_SPEED);
            leftBack.setPower(PIVOT_SPEED);
            if (getZAngle()< 60) {
                setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);
            } else {
                setPowerAll(-PIVOT_SPEED/2, -PIVOT_SPEED/2, PIVOT_SPEED/2, PIVOT_SPEED/2);
            }

            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }
        // Stop the robot
        setPowerAll(0, 0, 0, 0);
        timer.reset();
        while (timer.milliseconds() < 1000 && opModeIsActive()) {
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }

        // Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = getZAngle();

        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
       */
        double encoderDifference = Math.abs(leftPod.getCurrentPosition()) + (Math.abs(rightPod.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI*COUNTS_PER_INCH);

        horizontalTickOffset = horizontalPod.getCurrentPosition()/Math.toRadians(getZAngle());

        // Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            // Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            // Display raw values
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.addData("Vertical Left Position", -leftPod.getCurrentPosition());
            telemetry.addData("Vertical Right Position", rightPod.getCurrentPosition());
            telemetry.addData("Horizontal Position", horizontalPod.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            // Update values
            telemetry.update();
        }
    }

    private double getZAngle() {
        return (-imu.getAngularOrientation().firstAngle);
    }

    /**
     * Sets power to all four drive motors
     * @param rf power for right front motor
     * @param rb power for right back motor
     * @param lf power for left front motor
     * @param lb power for left back motor
     */
    private void setPowerAll(double rf, double rb, double lf, double lb){
        rightFront.setPower(rf);
        rightBack.setPower(rb);
        leftFront.setPower(lf);
        leftBack.setPower(lb);
    }
}
