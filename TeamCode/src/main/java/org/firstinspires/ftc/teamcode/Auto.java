package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.detection.objects.Rectangle;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Size;

import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.CENTER;
import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.LEFT;
import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.RIGHT;

/**
 * Created by main2 on 11/17/2017.
 */

public abstract class Auto extends LinearVisionOpMode {
    BNO055IMU imu;
    long timer = 0;
    VuforiaLocalizer vuforia;

    public int frameCount = 0;

    public int jewelLRedCounter = 0;
    public int jewelLBlueCounter = 0;
    public int jewelColor = 0;
    public int NO_COLOR = 0;
    public int RED_BLUE = 1;
    public int BLUE_RED = 2;

    //all servo and motors
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor centerDrive = null;
    public DcMotor liftDrive = null;
    public Servo leftServo = null;
    public Servo rightServo = null;
    public Servo armServoRed = null;
    public Servo armServoBlue = null;
    public DcMotor grabNabberLeft = null;
    public DcMotor grabNabberRight = null;
    public Servo relicRotation = null;
    public Servo relicGrabber = null;

    public VuforiaTrackables relicTrackables = null;
    public VuforiaTrackable relicTemplate = null;
    RelicRecoveryVuMark target = CENTER;
    public final int bluesideRightPos = -1910;
    public final int blueSideLeftPos = -530;
    public final int blueSideCenterPos = -1190;

    //encoder convertions

    //this is without any gear reductions.  MAKE SURE TO ACCOUNT FOR THIS!!!

    public void turnDegrees(double degrees, double correction, double minPower) {
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double error = 2;
        double degreesIMU;
        while (opModeIsActive() && (error < -1 || error > 1)) {

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            degreesIMU = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            error = degrees - degreesIMU;

            if (degrees > 0) {
                leftDrive.setPower(-((error / (330*correction)) + minPower));
                rightDrive.setPower((error / (330*correction)) + minPower);
                telemetry.addData("ERROR", error);
                telemetry.addData("IMUDEGREES", degreesIMU);
                telemetry.update();
            } else if (degrees < 0) {
                leftDrive.setPower(-((error / (330*correction)) - minPower));
                rightDrive.setPower((error / (330*correction)) - minPower);
                telemetry.addData("ERROR", error);
                telemetry.addData("IMUDEGREES", degreesIMU);
                telemetry.update();
            }
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        //composeTelemetry();

        // Wait until we're told to go
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public int jewelChecker() throws InterruptedException {
        while (opModeIsActive() && ((jewelLRedCounter < 150) && (jewelLBlueCounter < 150))) {

            if (beacon.getAnalysis().isLeftRed()){
                jewelLBlueCounter = 0;
                jewelLRedCounter++;
                jewelColor = RED_BLUE;
                telemetry.addData("Beacon Color", "RED THEN BLUE");
                telemetry.addData("counterR", jewelLRedCounter);

            }
            else if (beacon.getAnalysis().isRightRed()) {
                jewelLRedCounter = 0;
                jewelLBlueCounter++;
                jewelColor = BLUE_RED;
                telemetry.addData("Beacon Color", "BLUE THEN RED");
                telemetry.addData("counterB", jewelLBlueCounter);


            }
            else {
                telemetry.addData("Beacon Color", "NOT VISIBLE");
            }

            if (hasNewFrame()) {
                Mat rgba = getFrameRgba();
                Mat gray = getFrameGray();

                discardFrame();

                frameCount++;
            }

            waitOneFullHardwareCycle();
        }
        return jewelColor;
    }

    public void initJewelChecker(ScreenOrientation direction){
        this.setCamera(Cameras.PRIMARY);

        this.setFrameSize(new Size(900, 900));


        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control


        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        beacon.setAnalysisBounds(new Rectangle(new Point(width, height), width, 500));

        /**
         * Set color tolerances
         * 0 is default, -1 is minimum and 1 is maximum tolerance
         */
        beacon.setColorToleranceRed(-1);
        beacon.setColorToleranceBlue(-1);

        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(direction);


        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();
    }

    public void pictoChecker(){

        while (!opModeIsActive()) {

            //***********VUFORIA CODE*************
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            parameters.vuforiaLicenseKey = " ARP7n43/////AAAAGdGoThGE8k4YowI9EbuCTToRW5VvobacImha3msx7xPnmPUDGiTKRUxcagzehV6BEX4iIdhDpwbWDbKUxKCDvITmAR3E13KNy0uBnk61DpT4IrxJRDbLs3GkrxwSbrCK088QC2XY02KmMxY9kKMSNvg0HuNtB8IuFyo3wgIdkQhZOsDUpKM210oaIpRzyiz/XPez6hWYlfKJEUSGicvyhj1oR6++ey9mk6zKQDxRnSNJJqr6qTkdzs2aj0dV+7xZRpbFVTPO9Q4i955vzjdmS7Z6+FYVIqy8PGFKyT5YejHg/EYG+AJ4VHggr/+zS8YDZDgPpC0xgvTIh7uDz0idF7u3Jj0o68aPCCnqoGtaogTd ";

            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

            VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            VuforiaTrackable relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate");

            relicTrackables.activate();

            while (!opModeIsActive()) {
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark == RIGHT) {
                    telemetry.addData("VuMark", "%s visible", "RIGHT");
                    target = RIGHT;
                } else if (vuMark == LEFT) {
                    telemetry.addData("VuMark", "%s visible", "LEFT");
                    target = LEFT;
                } else if (vuMark == CENTER) {
                    telemetry.addData("VuMark", "%s visible", "CENTER");
                    target = CENTER;
                } else {
                    telemetry.addData("VuMark", "%s visible", "UNKNOWN");
                }

            }
        }
    }



    public void sideMoveToPos(int pos, double power){

        centerDrive.setTargetPosition(pos);
        centerDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        centerDrive.setPower(power);

        while (centerDrive.isBusy()){
            //wait until we reach the position
        }
        centerDrive.setPower(0);
        centerDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveInches(double inches){
        //through trial and error found that it's about 90 ticks per inch
        leftDrive.setTargetPosition((int)(inches*90)+ leftDrive.getCurrentPosition());
        rightDrive.setTargetPosition((int)(inches*90) + rightDrive.getCurrentPosition());

        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(0.3);
        rightDrive.setPower(0.3);

        while (leftDrive.isBusy() || rightDrive.isBusy()){
            //wait until we reach the position
            telemetry.addData("rightDrive", rightDrive.getCurrentPosition());
            telemetry.addData("leftDrive", leftDrive.getCurrentPosition());
            telemetry.update();

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
