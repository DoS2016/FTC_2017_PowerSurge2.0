package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
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

import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.CENTER;
import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.LEFT;
import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.RIGHT;
import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.UNKNOWN;

@Autonomous(name = "visionAuto")
public class LinearVisionSample extends LinearVisionOpMode {

    BNO055IMU imu;

    Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    Acceleration gravity  = imu.getGravity();

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    int frameCount = 0;

    int jewelLRedCounter = 0;
    int jewelLBlueCounter = 0;
    int jewelColor = 0;
    static int NO_COLOR = 0;
    static int RED_BLUE = 1;
    static int BLUE_RED = 2;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    RelicRecoveryVuMark Target = UNKNOWN;

        @Override
        public void runOpMode() throws InterruptedException {
            leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
            rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
            //Wait for vision to initialize - this should be the first thing you do
            waitForVisionStart();

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
                    Target = RIGHT;
                } else if (vuMark == LEFT) {
                    telemetry.addData("VuMark", "%s visible", "LEFT");
                    Target = LEFT;
                } else if (vuMark == CENTER) {
                    telemetry.addData("VuMark", "%s visible", "CENTER");
                    Target = CENTER;
                } else {
                    telemetry.addData("VuMark", "%s visible", "UNKNOWN");
                }

            }




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
            rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);


            cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
            cameraControl.setAutoExposureCompensation();

            waitForStart();

            while (opModeIsActive() && ((jewelLRedCounter < 200) && (jewelLBlueCounter < 200))) {
                //Log a few things
 /*               telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
                telemetry.addData("Beacon Center", beacon.getAnalysis().getLocationString());
                telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
                telemetry.addData("Beacon Buttons", beacon.getAnalysis().getButtonString());
                telemetry.addData("Screen Rotation", rotation.getScreenOrientationActual());
                telemetry.addData("Frame Rate", fps.getFPSString() + " FPS");
                telemetry.addData("Frame Size", "Width: " + width + " Height: " + height);
                telemetry.addData("Frame Counter", frameCount);*/

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
            Auto_Movement autoMovement = new Auto_Movement();
            autoMovement.moveToRight(leftDrive, rightDrive);

            BNO055IMU.Parameters parametersGyro = new BNO055IMU.Parameters();
            parametersGyro.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parametersGyro.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parametersGyro.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parametersGyro.loggingEnabled      = true;
            parametersGyro.loggingTag          = "IMU";
            parametersGyro.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parametersGyro);
            autoMovement.composeTelemetry(angles, gravity);

            /*            for (int i = 0; i < 3000; i = rightDrive.getCurrentPosition()){
                leftDrive.setPower(0.2);
                rightDrive.setPower(0.2);
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);*/



            //if(Target == RIGHT) {
                //if ((jewelColor == BLUE_RED) || (jewelColor == RED_BLUE)) {
                  //  telemetry.addData("Method", "Is Broken");
                  //  autoMovement.moveToRight(leftDrive, rightDrive);
                //}

//            }
    }
}
