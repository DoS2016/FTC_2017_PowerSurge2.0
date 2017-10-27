package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


/**
 * Created by main2 on 10/14/2017.
 */

public class Auto_Movement extends LinearVisionSample{



    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.75 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final int     COUNTS_PER_INCH         = (int) ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/
                (WHEEL_DIAMETER_INCHES * 3.14));

    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor centerDrive = null;
    //private DcMotor liftDrive = null;
    //private Servo leftServo = null;
    //private Servo rightServo = null;


    public void composeTelemetry(final Orientation angles, final Acceleration gravity) {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.

        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }
    //----------------------------------------------------------------------------------------------
    // Servo Formatting
    //----------------------------------------------------------------------------------------------

    public  String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void moveToRight(DcMotor leftDrive, DcMotor rightDrive) throws InterruptedException {

        final int RIGHT_SPACE_ENCODER = (34 * COUNTS_PER_INCH);


        for (int i = 0; i < RIGHT_SPACE_ENCODER; i = rightDrive.getCurrentPosition()){
            leftDrive.setPower(0.2);
            rightDrive.setPower(0.2);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);



    }

    public void turnDegrees(double degrees, DcMotor leftDrive, DcMotor rightDrive){
        while(angles.firstAngle != degrees){
            leftDrive.setPower((degrees + angles.firstAngle)/100);
            rightDrive.setPower(-((degrees + angles.firstAngle)/100));
        }
    }
        // Show the elapsed game time and wheel power.
    }

