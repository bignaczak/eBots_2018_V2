package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;

public abstract class eBotsOpMode extends LinearOpMode {

    //private Gyroscope imu;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    //Manipulation Motors
    public DcMotor armAngleMotor;
    public DcMotor armExtensionMotor;
    public DcMotor collectMotor;
    public DcMotor latchMotor;

    public BNO055IMU imu;          //Create a variable for the gyroscope on the Expansion Hub
    public double currentHeading;  //Angular direction in Degrees
    public double radHeading;       //Angular orientation in radians
    public Orientation angles;             //For imu gyroscope
    public Orientation radAngles;           //for radian heading
    public Acceleration gravity;           // State used for updating telemetry


    //Variables for the TensorFlow Object Detection
    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    public final static String VUFORIA_KEY = "AdGgXjv/////AAABmSSQR7vFmE3cjN2PqTebidhZFI8eL1qz4JblkX3JPyyYFRNp/Su1RHcHvkTzJ1YjafcDYsT0l6b/2U/fEZObIq8Si3JYDie2PfMRfdbx1+U0supMRZFrkcdize8JSaxMeOdtholJ+hUZN+C4Ovo7Eiy/1sBrqihv+NGt1bd2/fXwvlIDJFm5lJHF6FCj9f4I7FtIAB0MuhdTSu4QwYB84m3Vkx9iibTUB3L2nLLtRYcbVpoiqvlxvZomUd2JMef+Ux6+3FA3cPKCicVfP2psbjZrxywoc8iYUAq0jtsEaxgFdYoaTR+TWwNtKwJS6kwCgBWThcIQ6yI1jWEdrJYYFmHXJG/Rf/Nw8twEVh8l/Z0M";

    //  DEFINE CONSTANTS FOR THE ROBOT
    //  THESE ARE ALL POSITIONS ASSUMED THE ROBOT IS COMPLETELY FOLDED PRIOR TO START OF AUTON
    final static int ARM_ANGLE_COLLECT_POSITION = -11750;  //REAL POSITION
    //final static int ARM_ANGLE_COLLECT_POSITION = -10000;  //TEST POSITION
    final static int ARM_ANGLE_TRAVEL_POSITION = -5600;
    final static int ARM_ANGLE_SCORE_POSITION = -1000;    //MUST VERIFY
    final static int ARM_ANGLE_DUMP_POSITION = -9500;      //was -8000, -9500 was OK, but may catch side, -11000 might be too low, can hit glass

    //These are constants used to define counts per revolution of NEVEREST motors with encoders
    static final int NEVEREST_60_CPR = 1680;
    static final int NEVEREST_40_CPR = 1120;
    static final int NEVEREST_20_CPR = 560;

    //final int ARM_EXTENSION_COLLECTION_POSITION = 27800;
    final static int ARM_EXTENSION_COLLECTION_POSITION = -54750; //  54946 was observed in -57500;  //test position
    final static int ARM_EXTENSION_TRAVEL_POSITIION = -27800;
    final static int ARM_EXTENSION_DUMP_POSITION = -27600;

    final static int LATCH_DEPLOY_POSITION = -13800;        //-13800 is competition position, -12500 for eBots lander (shorter)
    //13500 is a little high for our practice lander
    final static int LATCH_DRIVE_POSITION = -5900;          //5900 is good, could be a little higher
    final static int LATCH_ENGAGE_POSITION = -11000;
    final static int LATCH_RISEUP_POSITION = -2500;

    //MOTOR PROTECTION ENCODER
    final static int ARM_ANGLE_LIMIT = -11750;  //REAL POSITION
    //final static int ARM_ANGLE_LIMIT = -10000;  //TEST POSITION
    final static int ARM_EXTENSION_LIMIT = -57500;
    final static int LATCH_LIMIT = -14000;

    final static long SAMPLING_DRIVE_TIME = 1350;
    final static long SAMPLING_EXTRA_DRIVE_TIME = 500;
    final static double SAMPLING_DRIVE_COMPONENT = 0.85;
    final static double SAMPLE_ALIGNMENT_TWIST_ANGLE = 30;
    final static long CENTER_SAMPLE_CRATER_DRIVE_ALIGNMENT_TIME = 1200;

    final static long DEPOT_DRIVE_TIME = 1100;
    final static long DEPOT_EXTRA_DRIVE_TIME = 0;
    final static double DEPOT_DRIVE_COMPONENT = .4;
    final static long CRATER_DRIVE_TIME = 2800;  //was 2500 for first successful sample
    final static long EXTRA_CRATER_DRIVE_TIME = 400;

    //****************************************************************
    //END CONSTANTS


    //TODO:  Refactor to mineral position
    enum GoldPosition
    {
        LEFT, CENTER, RIGHT, UNKNOWN;
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration for gyro
    //----------------------------------------------------------------------------------------------
    public void composeTelemetry() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();

            //  setting the currentHeading class variable
            currentHeading = angles.firstAngle;  //Convert to Radians
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
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //view context can be passed using View.getContext()
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void initializeImu(){
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    public void initializeDriveMotors(ArrayList<DcMotor> motorList){
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        //reverse direction for opposite side motors
        //This is needed based on how the motors are mounted on the robot
        //Clockwise vs. Counter Clockwise being forward
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        //Create an array of motors with their associated Power Setting
        //ArrayList<DcMotor> motorList= new ArrayList<>();
        motorList.add(frontLeft);
        motorList.add(frontRight);
        motorList.add(backLeft);
        motorList.add(backRight);

    }

    public void initializeManipMotors(){
        //Initialize motors for manipulator
        armAngleMotor = hardwareMap.get(DcMotor.class, "armAngleMotor");
        armAngleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armAngleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armAngleMotor.setDirection(DcMotor.Direction.REVERSE);

        armExtensionMotor = hardwareMap.get(DcMotor.class, "armExtensionMotor");
        armExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        collectMotor = hardwareMap.get(DcMotor.class, "collectMotor");

        latchMotor = hardwareMap.get(DcMotor.class, "latchMotor");
        latchMotor.setDirection(DcMotor.Direction.REVERSE);
        latchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        latchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public double getCurrentHeading(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = angles.firstAngle;
        return currentHeading;
    }

    public double getRadHeading(){
        radAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        radHeading=radAngles.firstAngle;
        return radHeading;

    }

    private static void calculateDriveVector(double driveMagnitude, double robotAngle, double spin, double[] outputArray){
        //Create an array of the drive values
        //[Element] --> Wheel
        //  [0] --> Front Left
        //  [1] --> Front Right
        //  [2] --> Back Left
        //  [3] --> Back Right
        outputArray[0] = driveMagnitude * Math.cos(robotAngle) + spin;
        outputArray[1] = driveMagnitude * Math.sin(robotAngle) - spin;
        outputArray[2] = driveMagnitude * Math.sin(robotAngle) + spin;
        outputArray[3] = driveMagnitude * Math.cos(robotAngle) - spin;

        //Now capture the max drive value from the array
        double maxValue = findMaxAbsValue(outputArray);

        //If any of the values exceed 1, then all drive values must be scaled
        //Divide all drive values by the max value to achieve a new max value of 1
        if (maxValue > 1) scaleDrive(1/maxValue, outputArray);
    }
    private static void calculateFieldOrientedDriveVector(double driveAngle ,double heading ,double drivePower, double spinPower, double[] outputArray){
        //This calculation returns drive vectors but relies on field orientated vectors
        //for driveAngle and currentHeading
        //Importantly, the rotationAngleGyroOriented is consistent with gyro, positive is spin to left
        //Spin is provided as a percentage of total value
        //[Element] --> Wheel
        //  [0] --> Front Left
        //  [1] --> Front Right
        //  [2] --> Back Left
        //  [3] --> Back Right
        double robotAngle = driveAngle - heading + Math.PI/4;
        //Note, now that spin is field orientated, indices 0&2 are negative while 1&3 are positive
        outputArray[0] = (Math.cos(robotAngle) * drivePower) - spinPower;
        outputArray[1] = Math.sin(robotAngle) * drivePower + spinPower;
        outputArray[2] = Math.sin(robotAngle) * drivePower - spinPower;
        outputArray[3] = Math.cos(robotAngle) * drivePower + spinPower;;

        //Don't scale the drive vector
    }

    private static void scaleDrive (double scaleFactor, double[] driveArray){
        for (int i=0; i<driveArray.length; i++) {
            driveArray[i] *= scaleFactor;
        }
    }

    private static double findMaxAbsValue(double[] array) {
        //Now capture the max drive value from the array
        double maxValue = Math.abs(array[0]);
        for (int i = 1; i < array.length; i++){
            if(Math.abs(array[i]) > maxValue) maxValue = Math.abs(array[i]);
        }
        return maxValue;
    }

    private static int findMaxAbsValue(int[] array) {
        //Now capture the max drive value from the array
        int maxValue = Math.abs(array[0]);
        for (int i = 1; i < array.length; i++){
            if(Math.abs(array[i]) > maxValue) maxValue = Math.abs(array[i]);
        }
        return maxValue;
    }

    private static double findMinAbsValue(double[] array) {
        //Now capture the max drive value from the array
        double minValue = Math.abs(array[0]);
        for (int i = 1; i < array.length; i++){
            if(Math.abs(array[i]) < minValue) minValue = Math.abs(array[i]);
        }
        return minValue;
    }

    private static int findMinAbsValue(int[] array) {
        //Now capture the max drive value from the array
        int minValue = Math.abs(array[0]);
        for (int i = 1; i < array.length; i++){
            if(Math.abs(array[i]) < minValue) minValue = Math.abs(array[i]);
        }
        return minValue;
    }

    public void performDriveStep(double xInput, double yInput, double spin, long duration, ArrayList<DcMotor> motors){
        //r gives the left stick's offset from 0 position by calculating hypotenuse of x and y offset
        double speed = Math.hypot(xInput, yInput );

        //Robot angle calculates the angle (in radians) and then subtracts pi/4 (45 degrees) from it
        //The 45 degree shift aligns the mecanum vectors for drive
        double robotAngle = Math.atan2(yInput, xInput) - Math.PI / 4;

        double[] driveValues = new double[4];
        calculateDriveVector(speed, robotAngle, spin, driveValues);     //Calculate motor drive speeds

        //Setup the time increment for autonomous
        long currentTime = System.nanoTime() / 1000000;  //current time in milliseconds
        for(long t=currentTime; t < (currentTime+duration); t = (long) (System.nanoTime() / 1000000)){
            //Now actually assign the calculated drive values to the motors in motorList
            int i=0;
            for (DcMotor m: motors){
                m.setPower(driveValues[i]);
                i++;
            }
            telemetry.addData("Speed", speed);
            telemetry.addData("Angle", robotAngle);
            telemetry.addData("spin", spin);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
        //Stop all the motors
        stopMotors(motors);
    }
    public void twistToAngle(double spinAngleInDegrees, double speed, ArrayList<DcMotor> motors){
        //Note this logic is demoed in the "Gyro" tab of "Rover Time Trials" Google Sheets file
        //All angles in this routine are in DEGREES
        boolean currentHeadingModifier = false;  //Used as a flag if the target angle passes the 180 point
        double overFlowAngle = 0;
        double adjustedAngle = getCurrentHeading();  //Adjusted angle is to handle crossover of sign at 180 degrees
        double angleBufferForPrecision = 35;
        double fullSpeed = 0.5;
        double throttleDownSpeed = 0.2;
        double slopeForThrottleDown = (fullSpeed-throttleDownSpeed)/angleBufferForPrecision;

        //Create loop so robot spins until target angle is achieved
        //Based on the field coordinate system, positive roll is spinning to the left
        //But the drive vector equations consider turning to the right to be positive
        //So to establish a targetAngle, the desired spinAngleInDegrees must be subtracted from the currentHeading
        //But before target angle, make sure that the spin angle is efficient, don't let it exceed 180


        if(Math.abs(spinAngleInDegrees)>180){
            spinAngleInDegrees = (360 - Math.abs(spinAngleInDegrees))*(-Math.signum(spinAngleInDegrees));
        }

        double targetAngle = adjustedAngle - spinAngleInDegrees;

        //The imu changes sign at 180 degrees
        // For this control loop to work, must catch this CROSSOVER event
        if (Math.abs(targetAngle)>180){
            currentHeadingModifier = true;
            if(targetAngle>180) targetAngle = -360+targetAngle;
            if(targetAngle<-180) targetAngle = targetAngle+360;
        }

        //Now that that the target angle has been normalized, it is possible that
        //The current heading will have a different sign from target angle
        //Detect if that is the case and normalize the adjusted angle
        //But be careful, this happens from angle -10 to 10 as well as -179 to 170
        //So this only happens when turning in a direction and signs are different unexpectedly
        // (like angles increasing but target is negative when starting positive)
        //  Think of starting at 179 an spinning CCW (-2) to -179
        //  You would expect angle to increase, but target is less than starting point
        if ((spinAngleInDegrees>0 && targetAngle > currentHeading) |//Spinning to the right, so angle count is decreasing AND target angle > currentHeading(adjustedAngle)
                (spinAngleInDegrees<0 && targetAngle < currentHeading)){
            adjustedAngle= (360-Math.abs(currentHeading))*Math.signum(targetAngle);
        } else{
            adjustedAngle = currentHeading;
        }

        double angleFromTarget = Math.abs(spinAngleInDegrees);
        if(angleFromTarget < angleBufferForPrecision){
            //This formula was checked on Google Sheets
            speed = fullSpeed - ((slopeForThrottleDown) * (angleBufferForPrecision-angleFromTarget));
        }else{
            speed = fullSpeed;
        }
        startSpinning(spinAngleInDegrees, speed, motors);
        double spinAngleUndershoot = 2;
        //TODO: Compress these two loops into 1
        if(spinAngleInDegrees > 0){
            //If spinning to the right, keep spinning while angle is greater than target angle
            // When spinning right, imu angle is decreasing
            while(opModeIsActive() && adjustedAngle > (targetAngle+spinAngleUndershoot)) {
                //  Just keep spinning to the right

                //Get the new adjusted angle
                getCurrentHeading();
                if ((spinAngleInDegrees>0 && targetAngle > currentHeading) |//Spinning to the right, so angle count is decreasing AND target angle > currentHeading(adjustedAngle)
                        (spinAngleInDegrees<0 && targetAngle < currentHeading)){
                    adjustedAngle= (360-Math.abs(currentHeading))*Math.signum(targetAngle);
                } else{
                    adjustedAngle = currentHeading;
                }


                //If close to the target angle
                angleFromTarget = Math.abs(adjustedAngle - targetAngle);
                if (angleFromTarget < angleBufferForPrecision){
                    speed = fullSpeed - ((slopeForThrottleDown) * (angleBufferForPrecision-angleFromTarget));
                    startSpinning(spinAngleInDegrees,speed, motors);
                }

                telemetry.addData("Target Spin", spinAngleInDegrees);
                telemetry.addData("Target Angle", targetAngle);
                telemetry.addData("currentHeading", adjustedAngle);
                telemetry.addData("Status", "Turning");
                telemetry.update();
            }
        }else{
            //If spinning to the left, keep spinning while angle is less than target angle
            //When spinning left, the imu angle is increasing
            //so keep spinning while heading is less than target
            while (opModeIsActive() && adjustedAngle < (targetAngle-spinAngleUndershoot)) {
                //  Just keep spinning to the left

                //Get the new adjusted angle
                getCurrentHeading();
                //Get the new adjusted angle
                getCurrentHeading();
                if ((spinAngleInDegrees>0 && targetAngle > currentHeading) |//Spinning to the right, so angle count is decreasing AND target angle > currentHeading(adjustedAngle)
                        (spinAngleInDegrees<0 && targetAngle < currentHeading)){
                    adjustedAngle= (360-Math.abs(currentHeading))*Math.signum(targetAngle);
                } else{
                    adjustedAngle = currentHeading;
                }

                angleFromTarget = Math.abs(adjustedAngle - targetAngle);
                if (Math.abs(adjustedAngle - targetAngle) < angleBufferForPrecision){
                    speed = fullSpeed - ((slopeForThrottleDown) * (angleBufferForPrecision-angleFromTarget));
                    startSpinning(spinAngleInDegrees,speed, motors);
                }

                telemetry.addData("Target Spin", spinAngleInDegrees);
                telemetry.addData("Target Angle", targetAngle);
                telemetry.addData("currentHeading", adjustedAngle);
                telemetry.addData("Status", "Turning");
                telemetry.update();
            }
        }
        telemetry.addData("Status", "Spin Complete");
        telemetry.update();

        //  Now stop
        stopMotors(motors);
    }


    private void startSpinning(double spinAngle, double speed,  ArrayList<DcMotor> motors){
        //Using similar inputs as the calculateDriveVectors routine
        //Spin variable is assigned based on the sign of the spinAngle
        //If spinning to the right, no need to change sign
        double spin;
        if (spinAngle>=0){
            spin = speed;
        } else {
            spin = -speed;
        }

        //Instead of calling the calculateDriveVector function just apply the appropriate spin
        //The magnitude and robotAngle are irrelevant for this
        double[] driveValues = new double[4];
        driveValues[0] = spin;
        driveValues[1] =  - spin;
        driveValues[2] = spin;
        driveValues[3] = - spin;

        //Set the robot in motion in the spin
        int i=0;
        for (DcMotor m: motors){
            m.setPower(driveValues[i]);
            i++;
        }
    }
    private void stopMotors(ArrayList<DcMotor> motors) {

        long stopTime = 150;
        long currentTime = System.nanoTime() / 1000000;
        for (long t = currentTime; t < (currentTime + stopTime); t = (System.nanoTime() / 1000000)) {
            for (DcMotor m : motors) {
                m.setPower(0);
            }
        }
    }

    public boolean turnToFieldHeading (double desiredFieldHeadingInDegrees, ArrayList<DcMotor> motors){
        // Field heading is the imu direction based on the assumed zero point from lander
        // This command calculates the turn inputs for twistToAngle function and calls it
        // Angles in this routine are in DEGREES (unless otherwise noted)
        double requiredTurnAngle = checkHeadingVersusTarget(Math.toRadians(desiredFieldHeadingInDegrees));  //How many angles must turn
        requiredTurnAngle = Math.toDegrees(requiredTurnAngle);
        twistToAngle(requiredTurnAngle,0.35, motors);

        //Note:  turnError is initially returned in Radians
        double turnError = checkHeadingVersusTarget(Math.toRadians(desiredFieldHeadingInDegrees));
        turnError = Math.toDegrees(turnError);  //Now converted to degrees

        if(Math.abs(turnError)<5){
            return true;
        }else{
            return false;
        }
    }

    private void performDriveStepTimedTranslateAndSpin(ArrayList<DcMotor> motors, double driveAngle, double driveTime, double spinPower, double drivePower, double[] driveValues, double headingTarget) {
        int[] currentMotorPositions = new int[4];
        double maxSlowDownFactor = 0.5;
        long transientFence = 500;  //accelerate and decelerate within this time window
        getRadHeading();
        double maxScale = 0.9;   //was 0.65
        if(driveTime<1) maxScale=0.65;  //If a delicate maneuveur, go slow
        double driveScaleFactor=1;
        long currentTime=(System.nanoTime()/1000000);
        long startTime = currentTime;
        long endTime = startTime +  (long) (driveTime*1000 * (1/maxScale));  //Add buffer for transientFence
        long elapsedTime=0;
        long remainingTime=endTime;
        long transientComparison = 0;
        double transientScale = 1;
        while(opModeIsActive() && (currentTime < endTime)){
            calculateFieldOrientedDriveVector(driveAngle,radHeading,drivePower,spinPower,driveValues);
            driveScaleFactor = maxScale/findMaxAbsValue(driveValues);  //Normalize drive vector, make sure nothing > 1
            scaleDrive(driveScaleFactor,driveValues);
            /*//Apply transient scale if within fence
            if(transientComparison<transientFence) {
                transientScale = (transientFence - transientComparison)/transientFence;  //Percentage of slowdown to use
                transientScale = transientScale*(1-maxSlowDownFactor);  //when transientComparison at 0, scale to down to 0.3
                scaleDrive(transientScale,driveValues);

            }*/
            for(int i = 0; i<motors.size(); i++){
                motors.get(i).setPower(driveValues[i]);
            }
            currentTime=(System.nanoTime()/1000000);
            elapsedTime=currentTime-startTime;
            remainingTime= endTime-currentTime;
            transientComparison = (elapsedTime < remainingTime) ? elapsedTime : remainingTime;

            getRadHeading();
            getMotorPositions(currentMotorPositions, motors);
            telemetry.addData("drivePower", drivePower);
            telemetry.addData("driveTime", driveTime);
            telemetry.addData("Heading", Math.toDegrees(radHeading));
            telemetry.addData("radHeading", radHeading);
            telemetry.addData("driveValues", Arrays.toString(driveValues));
            telemetry.addData("Actual Value0", Arrays.toString(currentMotorPositions));
            telemetry.update();
        }
        double headingError = checkHeadingVersusTarget(headingTarget);
        if(Math.abs(headingError)>Math.toRadians(5)) {
            twistToAngle(Math.toDegrees(headingError), 0.2, motors);
        }
        stopMotors(motors);
//        endTime = System.nanoTime()/1000000 + (long) (4*1000);

//        while(opModeIsActive() && System.nanoTime()/1000000 < endTime) {
//            telemetry.addData("drivePower", drivePower);
//            telemetry.addData("driveTime", driveTime);
//            telemetry.addData("Heading", Math.toDegrees(radHeading));
//            telemetry.addData("radHeading", radHeading);
//            telemetry.addData("driveValues", Arrays.toString(driveValues));
//            telemetry.addData("Actual Value0", Arrays.toString(currentMotorPositions));
//            telemetry.update();
//
//        }
    }

    public boolean moveByDistance(double inchesForward, double inchesLateral,
                                  double rotationAngleGyroOriented, ArrayList<DcMotor> motors,
                                  String selectedDriveType){
        //  For this method, forward travel is relative to robot heading of heading 0 (gyro initialize position)
        //  For rotationAngleGyroOriented, positive spin is towards left, negative spin is towards right
        //  Units for rotationAngleGyroOriented is Radians
        //  NOTE:  this spin is opposite of other occurrences in this app, but in alignment with gyro
        //  It requires a change in how the spin affects drive vectors
        //  Wheel orientation assumes X orientation when looking from above down to ground plane

        //  Convert units for rotationAngleGyroOriented to radians if greater than 2*pi, assume it was passed as degrees
        if (rotationAngleGyroOriented>(2*Math.PI)) rotationAngleGyroOriented= Math.toRadians(rotationAngleGyroOriented);

        //Analyze the move and determine how much distance each wheel must travel
        //  Calculate the drive power vector for the motors
        //  Determine the target heading
        //  Determine time to complete

        //Initialize variables for drive distance calculations
        int encoderClicksPerRevolution = NEVEREST_40_CPR;
        double wheelDiameter = 4.0;
        double wheelDiagonalSpacing = 20.69;
        double peakRobotSpeed = 30;  //peak robot speed in inches per second
        double wheelAngle = (Math.PI)/4;
        // This is net speed in forward or sideways direction, but wheels must travel faster due to slippage
        // So apply correction factor by dividing by cos(wheelAngle), or cos(pi/4) for pure slip condition
        // However, this variable will need to be verified.  Based on spin timing, assume no slip initially
        //TODO:  Feather-in this slip factor
        //TODO:  It appears to be a function of the alignment of driveVector with wheelAngle
        //TODO:  FOR Lateral movement, the spin efficiency is effectively 1, for spin it is 1/sqrt(2)
        double wheelSlipFactor = Math.cos(0);  //  This could be as high as Math.cos(Math.PI/4)
        double peakWheelSpeedInchesPerS = peakRobotSpeed / wheelSlipFactor;  //  in/s
        int peakWheelSpeedClicksPerS = (int) Math.round((peakWheelSpeedInchesPerS / (Math.PI*wheelDiameter))*encoderClicksPerRevolution);  //  clicks/s

        //Note:  the theoretical effectiveClicksPerInch was close (89 compared to 93 empirical)
        //       But the number was way off for rotation
        //int effectiveClicksPerInchLateral = (int) Math.round(peakWheelSpeedClicksPerS/peakRobotSpeed);
        int effectiveClicksPerInchLateral = 93;   //empirically derived
        int effectiveClicksPerInchRotation = 125;   //empirically derived

        //This boost factor is defined to increase the amount of spin factor in drive vector
        double spinBoostFactor = (double) effectiveClicksPerInchRotation/effectiveClicksPerInchLateral;

        //TODO:  Consider the effect of the spinBoostFactor and changing peakAngularVelocity
        //TODO:  I added a factor of 1/sqrt(2) (-.707) to the angular velocity speed
        double peakAngularVelocity = (2*peakWheelSpeedInchesPerS)/(wheelDiagonalSpacing*Math.sqrt(2));

        //Calculate wheel revolution distance for travel
        //Note:  This is field-oriented, Gyro Zero (set at hanging) defines orientation of positive X direction
        //Therefore the atan2 function in this routine uses the lateral for the (first) y component and forward is (second) x
        //And to calculate robot angle, use driveAngle - currentHeading + wheelAngle (or pi/4)
        double driveAngle = Math.atan2(inchesLateral,inchesForward);   //  Angle between pi & -pi used for mecanum drive vector
        //To handle crossover between driveAngle and currentHeading, use the checkHeadingVersusTarget function
        double travelDistance = Math.sqrt(Math.pow(inchesForward,2) + Math.pow(inchesLateral,2));
        //And then add wheelAngle to that

        //Set the target heading.  Note that rotationAngleGyroOriented is positive if to left (same as gyro)
        //That is why rotationAngleGyroOriented is added to currentHeading
        //Also note that the target heading can have abs value greater than 180, which may be a problem later
        getCurrentHeading();  //Refreshes the currentHeading class attribute
        double targetHeading = Math.toRadians(currentHeading)+rotationAngleGyroOriented;

        //TODO:  Can this be replaced with checkHeadingVersusTarget?
        //Deal with angle overflow (abs(angle)>180)
        if (targetHeading > Math.toRadians(180)){
            targetHeading = -Math.toRadians(180) - (Math.toRadians(180)-targetHeading);  //subtracting a negative number from -180 reduces magnitude
        }else if (targetHeading < -Math.toRadians(180)){
            targetHeading = Math.toRadians(180)-(-Math.toRadians(180)-targetHeading);  //note targetHeading is negative and larger magnitude, resulting in positive number
        }

        //  t=(wheelDiam / peakWheelSpeedInchesPerS) + ((rotationAngleGyroOriented)/(peakAngularVelocity)
        double expectedTravelTime = (travelDistance/peakWheelSpeedInchesPerS) + (rotationAngleGyroOriented/
                peakAngularVelocity);
        //Based on robot peak speeds, allocate power ratios to the rotate and translate operations
        double spinPower = Math.abs(rotationAngleGyroOriented)/(expectedTravelTime*peakAngularVelocity);
        double drivePower = 1-spinPower;
        double rotationRateForDriveStep = rotationAngleGyroOriented/expectedTravelTime;
        double requiredSpinDistance = wheelDiagonalSpacing/2 * Math.abs(rotationAngleGyroOriented);  //s=r*theta
        //Note, drive values assume the following order of wheels  [FL, FR, BL, BR]
        //  This calculation is used to determine distances, and isn't the final calc for motors
        double[] driveValues = new double[4];
        //Note:  At this step of the process, the drive values are being used to calculate
        //      distances and encoder counts.  Note,  drivePower scale factor is included
        //  TODO:  Verify that drivePower doesn't need to be set to 1
        calculateFieldOrientedDriveVector(driveAngle,Math.toRadians(currentHeading),drivePower, 0,driveValues);

        //Using the drive values, drive distances can be calculated
        //First figure out the length that each wheel must travel to translate
        //Then add the distance required to spin
        double[] driveDistances = new double[4];    //Array to capture the distance values in inches
        int[] encoderTargetValues = new int[4];     //Array to capture encoder clicks
        int[] translationClicks = new int[4];
        int[] rotationClicks = new int[4];
        double[] coefOutputForDebug=new double[4];
        for (int i=0; i<4; i++){
            //First get the coefficients, which are calculated differently if spin is included
            if(rotationAngleGyroOriented==0) {
                driveDistances[i] = driveValues[i]*expectedTravelTime;
            }else{
                double coef_1;
                double coef_2;
                double termInsideOperand_1 = -(rotationRateForDriveStep * expectedTravelTime) + driveAngle + wheelAngle;
                double termInsideOperand_2 = driveAngle + wheelAngle;
                //TODO:  Fix this calculation
                if (i == 0 | i == 3) {
                    coef_1 = -Math.sin(termInsideOperand_1) / rotationRateForDriveStep;
                    coef_2 = -Math.sin(termInsideOperand_2) / rotationRateForDriveStep;

                } else {
                    coef_1 = Math.cos(termInsideOperand_1) / rotationRateForDriveStep;
                    coef_2 = Math.cos(termInsideOperand_2) / rotationRateForDriveStep;
                }
                //must include consideration for drivePower here, since already included in no-spin variant
                driveDistances[i] = (coef_1 - coef_2) * drivePower;
            }
            coefOutputForDebug[i] = driveDistances[i];

            //Multiply the coefficients to determine drive distances
            //Note:  time element of the distance is already included in the coefficient
            //      Also, the drivePower has been factored in when calculating driveValues for the no spin option
            driveDistances[i] = driveDistances[i] * peakWheelSpeedInchesPerS ;
            //If the robot is spinning, must add the spin distance
            //Based on wheel configuration, spin is added respectively [FL, FR, BL, FR] --> [-1, 1, -1, 1]  NOTE:  reversed from calculate drive vectors due to opposite spin direction
            //Convert the distances to encoder targets
            //  Front motors increase clicks with positive power
            //  Rear motors decrease clicks with positive power
            //  Spin requires ~122 clicks per inch circumference traveled
            //  Lateral movement requires ~93
            translationClicks[i] = (int) (driveDistances[i] * effectiveClicksPerInchLateral);
            int motorRotationSign=1;
            if (i==0 | i==2) {motorRotationSign =  -1;}  //subtract for index 0 and 2, add for 1 and 3
            rotationClicks[i] = (int) (requiredSpinDistance*motorRotationSign*effectiveClicksPerInchRotation);
            encoderTargetValues[i] = translationClicks[i]+rotationClicks[i];
            if(i==2 | i==3){
                //Reverse the sign for back motors
                encoderTargetValues[i] *= -1;
            }
        }

        //Now perform the drive step
        //The drive portion has been refactored and different variants are allowed
        boolean driveStepPositionReached = false;
        boolean driveStepTimedOut = false;
        boolean driveStepOvershootDetected = false;

        if(selectedDriveType.equalsIgnoreCase("Custom")) {
            PerformDriveStepCustomEncoderControl customDriveStep = new PerformDriveStepCustomEncoderControl(motors, encoderClicksPerRevolution, spinBoostFactor, driveAngle, expectedTravelTime, spinPower, drivePower, requiredSpinDistance, driveValues, encoderTargetValues).invoke();
            driveStepPositionReached = customDriveStep.isDriveStepPositionReached();
            driveStepTimedOut = customDriveStep.isDriveStepTimedOut();
            driveStepOvershootDetected = customDriveStep.isDriveStepOvershootDetected();
        }else if(selectedDriveType.equalsIgnoreCase("RunToPosition")) {
            //TODO:  Figure out why motors spin wrong way, probably reversed polarity in voltage wiring
            //This is an alternate drive method that could be utilized
            performDriveStepRunToPosition(motors, driveAngle, spinPower, drivePower, driveValues, encoderTargetValues);

            //Stop all motors
            stopMotors(motors);
        }else if(selectedDriveType.equalsIgnoreCase("TimedTranslateAndSpin")) {
            //Recalculate travel time using simplified method
            spinPower=requiredSpinDistance/(travelDistance+requiredSpinDistance);
            drivePower = 1-spinPower;
            if (rotationAngleGyroOriented<0) spinPower*=-1;
            double extraTimeForAcceleration = 0.25;
            expectedTravelTime = (travelDistance)/(peakRobotSpeed*drivePower) + extraTimeForAcceleration;
            performDriveStepTimedTranslateAndSpin(motors,driveAngle,expectedTravelTime,spinPower,drivePower,driveValues, targetHeading);

        }else if(selectedDriveType.equalsIgnoreCase("Debug")){
            //Don't perform any drive step
            //Print the calculations to terminal
            while(opModeIsActive()) {
                telemetry.addData("Coef", Arrays.toString(coefOutputForDebug));
                telemetry.addData("Calculated Travel Time", expectedTravelTime);
                telemetry.addData("Spin Power", spinPower);
                telemetry.addData("drivePower", drivePower);
                telemetry.addData("Heading", currentHeading);
                telemetry.addData("radHeading", getRadHeading());
                telemetry.addData("driveAngle", driveAngle);
                telemetry.addData("driveValues", Arrays.toString(driveValues));
                telemetry.addData("spinDistance", "%.2f", requiredSpinDistance);
                telemetry.addData("translateClicks", Arrays.toString(translationClicks));
                telemetry.addData("rotationClicks", Arrays.toString(rotationClicks));
                telemetry.addData("targets", Arrays.toString(encoderTargetValues));
                telemetry.update();
            }

        }

        //Check the heading at the end of the move and correct it if necessary
        double headingError = checkHeadingVersusTarget(targetHeading);
        double targetHeadingAccuracy = Math.toRadians(5);
        boolean targetHeadingAchieved = false;
        if (Math.abs(headingError) <= targetHeadingAccuracy) targetHeadingAchieved=true;
        //If error is too high, correct
        /*if (Math.abs(headingError)>10)  {
            twistToAngle(headingError,0.2,motors);
        }*/

        //Now check all the different states of the drive step to determine if it was successfully completed

        if (driveStepPositionReached && targetHeadingAchieved
                && !driveStepTimedOut && !driveStepOvershootDetected) {
            return true;
        }else{
            return false;
        }

        //return true;
    }
    private void performDriveStepRunToPosition(ArrayList<DcMotor> motors, double driveAngle, double spinPower, double drivePower, double[] driveValues, int[] encoderTargetValues) {
        getCurrentHeading();
        calculateFieldOrientedDriveVector(driveAngle, Math.toRadians(currentHeading), drivePower, spinPower, driveValues);
        for(int i=0; i<motors.size(); i++) {
            motors.get(i).setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors.get(i).setTargetPosition(encoderTargetValues[i]);
            motors.get(i).setPower(0.1);
        }

        int[] currentMotorPositions = new int[4];
        while(opModeIsActive() &&
                (motors.get(0).isBusy() | motors.get(1).isBusy()
                        | motors.get(2).isBusy() | motors.get(3).isBusy())){

            getCurrentHeading();
            getRadHeading();
            getMotorPositions(currentMotorPositions, motors);

            telemetry.addData("heading", currentHeading);
            telemetry.addData("radHeading", radHeading);
            telemetry.addData("target values", Arrays.toString(encoderTargetValues));
            telemetry.addData("target values", Arrays.toString(encoderTargetValues));
            telemetry.addData("Actual Values", Arrays.toString(currentMotorPositions));
            telemetry.update();

        }
    }

        private double checkHeadingVersusTarget(double targetHeadingInRadians){
        //Check the heading at the end of the move and correct it if necessary
        //target heading should be in radians
        //Return value is in radians
        //The sign of the return angle, when considering gyro-oriented vectors (CCW is positive)
        //uses the target as the basis and error value orients to the current heading
        //so the if target is 0 and heading is -pi/2, the return is -pi/2
        getCurrentHeading();
        double headingError = Math.toRadians(currentHeading)-targetHeadingInRadians;
        //If large error, assume that crossover has occurred
        if (Math.abs(headingError)>Math.toRadians(180)){
            //Correct the heading error caused by crossover
            //This math was tested in Google Sheets
            headingError = (2*Math.PI - Math.abs(headingError)) * Math.signum(targetHeadingInRadians);
        }
        return headingError;  //Note, this is returned in Radians
    }

    private void getMotorPositions(int[] currentPositions, ArrayList<DcMotor> motors) {
        for(int i=0; i<currentPositions.length; i++){
            currentPositions[i] = motors.get(i).getCurrentPosition();
        }
    }

    public void lowerLatchToDrivePosition(){
        latchMotor.setTargetPosition(LATCH_DRIVE_POSITION);
        //latchMotor.setTargetPosition(0);
        latchMotor.setTargetPosition(LATCH_DEPLOY_POSITION);
        latchMotor.setPower(1);
    }

    public void moveArmToDumpPosition(){

        armAngleMotor.setTargetPosition(ARM_ANGLE_DUMP_POSITION);
        armAngleMotor.setPower(1);

    }
    public void extendArmToClaim(){
        armExtensionMotor.setTargetPosition(ARM_EXTENSION_DUMP_POSITION);
        armExtensionMotor.setPower(1);
    }


    public void extendArm(){
        armExtensionMotor.setTargetPosition(ARM_EXTENSION_COLLECTION_POSITION);
        armExtensionMotor.setPower(1);
    }

    public void waitForArmsToMove(){
        while(opModeIsActive()
                && (armExtensionMotor.isBusy()
                | armAngleMotor.isBusy()
                | latchMotor.isBusy())){
            //Just wait
        }
    }

    public void depositMarkerInDepot(ArrayList<DcMotor> motors){
        long endTimer = System.nanoTime()/1000000 + 5000;//Set timer for 5 seconds

        while(opModeIsActive()
                && armAngleMotor.isBusy()
                && System.nanoTime()/1000000<endTimer){
            //Wait for the marker to be placed
            //We could add a touch sensor here to improve timing
            telemetry.update();
        }
        armAngleMotor.setPower(0);
        //  b) Spit out the marker
        collectMotor.setPower(-1);
        long delayTime = 800;
        performDriveStep(0, 0, 0, delayTime, motors);  //delay for a second
        collectMotor.setPower(0);       //turn off the motor
    }

    public void moveArmtoTravelPosition(){
        armAngleMotor.setTargetPosition(ARM_ANGLE_TRAVEL_POSITION);
        armAngleMotor.setPower(1);

    }


    public GoldPosition landAndLocateGoldMineral(){
        if (tfod != null) {
            tfod.activate();
        }


        latchMotor.setTargetPosition(LATCH_DEPLOY_POSITION);
        latchMotor.setPower(1);

        //Initialize variables for the sampling loop
        int samplingScanCount = 0;
        int goldLeftCount = 0;
        int goldCenterCount = 0;
        int goldRightCount = 0;
        int samplingResponseReceived = 0;
        double samplingInputPercentage = 0;
        double goldLeftConfidence = 0;
        double goldCenterConfidence = 0;
        double goldRightConfidence = 0;
        double thresholdConfidence = 0.5;
        GoldPosition goldPosition = GoldPosition.UNKNOWN;
        boolean goldPositionDetermined = false;



        while(latchMotor.isBusy()) {
            //while the robot is descending, scan for minerals
            if (tfod != null) {
                TensorFlowRefactor tensorFlowRefactor = new TensorFlowRefactor().invoke();
                samplingScanCount++;    //Increment the number of samping scans
                //Now number of loops where sampling response is received
                if (tensorFlowRefactor.getGoldPosition() != GoldPosition.UNKNOWN
                        | tensorFlowRefactor.getGoldNotLocated() != GoldPosition.UNKNOWN) {
                    samplingResponseReceived++;
                }
                //Now use the results from the scan to record observations
                if (tensorFlowRefactor.getGoldPosition() == GoldPosition.LEFT) goldLeftCount++;
                else if (tensorFlowRefactor.getGoldPosition() == GoldPosition.CENTER)
                    goldCenterCount++;
                else if (tensorFlowRefactor.getGoldPosition() == GoldPosition.RIGHT)
                    goldRightCount++;

                if (tensorFlowRefactor.getGoldNotLocated() == GoldPosition.LEFT) goldLeftCount--;
                else if (tensorFlowRefactor.getGoldNotLocated() == GoldPosition.CENTER)
                    goldCenterCount--;
                else if (tensorFlowRefactor.getGoldNotLocated() == GoldPosition.RIGHT)
                    goldRightCount--;

            }
            if (samplingResponseReceived > 0) {
                goldLeftConfidence = (double) (goldLeftCount) / samplingResponseReceived;
                goldCenterConfidence = (double) (goldCenterCount) / samplingResponseReceived;
                goldRightConfidence = (double) (goldRightCount) / samplingResponseReceived;
                samplingInputPercentage = (double) (samplingResponseReceived) / samplingScanCount;

            }
            telemetry.addData("Scan Count", samplingScanCount);
            telemetry.addData("Input Rate", samplingInputPercentage);
            telemetry.addData("Left Rating", goldLeftConfidence);
            telemetry.addData("Center Rating", goldCenterConfidence);
            telemetry.addData("Right Rating", goldRightConfidence);
            telemetry.update();
        }

        //Based on the confidence numbers calculated above, determine the final goldPosition
        if (goldLeftConfidence>goldCenterConfidence
                && goldLeftConfidence > goldRightConfidence
                && goldLeftConfidence > thresholdConfidence){
            goldPosition = GoldPosition.LEFT;
        }else if (goldCenterConfidence>goldLeftConfidence
                && goldCenterConfidence > goldRightConfidence
                && goldCenterConfidence > thresholdConfidence){
            goldPosition = GoldPosition.CENTER;
        }else if(goldRightConfidence>goldLeftConfidence
                && goldRightConfidence > goldCenterConfidence
                && goldRightConfidence>thresholdConfidence){
            goldPosition = GoldPosition.RIGHT;
        } else {goldPosition = GoldPosition.UNKNOWN;}

        //If position was determined, set the boolean flag
        if (goldPosition != GoldPosition.UNKNOWN) goldPositionDetermined = true;

        //if gold location is not determined, then randomly assign a location
        if(!goldPositionDetermined){
            double randomNumber = Math.random();
            if(randomNumber <= 0.33){
                goldPosition = GoldPosition.LEFT;
            } else if (randomNumber <=0.67){
                goldPosition = GoldPosition.CENTER;
            } else{
                goldPosition = GoldPosition.RIGHT;
            }
        }
        //Report out the final position
        telemetry.addData("Gold Position", goldPosition.toString());
        telemetry.update();


        //Cut power to the latch motor
        latchMotor.setPower(0);

        return goldPosition;
    }


    public class TensorFlowRefactor {
        private boolean goldRelativePositionDetermined;
        private GoldPosition goldRelativePosition;
        private GoldPosition goldAbsolutePosition;
        private GoldPosition silver1AbsolutePosition;
        private GoldPosition silver2AbsolutePosition;
        private GoldPosition goldNotLocated;
        private int goldMineralX;
        private int silverMineral1X;
        private int silverMineral2X;
        private double absoluteConfidence;
        private int itemCount;

        public TensorFlowRefactor() {
            this.goldRelativePositionDetermined = false;
            this.goldRelativePosition = GoldPosition.UNKNOWN;
            this.goldAbsolutePosition = GoldPosition.UNKNOWN;
            this.silver1AbsolutePosition = GoldPosition.UNKNOWN;
            this.silver2AbsolutePosition = GoldPosition.UNKNOWN;
            this.goldNotLocated = GoldPosition.UNKNOWN;
            this.goldMineralX = -1;
            this.silverMineral1X = -1;
            this.silverMineral2X = -1;
            this.absoluteConfidence = 0;
        }

        //The getters
        public boolean isGoldPositionDetermined() {
            return goldRelativePositionDetermined;
        }

        public GoldPosition getGoldPosition() {
            //If gold position determined from seeing all three objects, use that answer
            //Otherwise, return the result of the absolute position
            if(goldRelativePosition != GoldPosition.UNKNOWN) {
                return goldRelativePosition;
            } else {
                return goldAbsolutePosition;
            }
        }
        public GoldPosition getGoldNotLocated() {return goldNotLocated;}
        public int getGoldMineralX(){return goldMineralX;}
        public int getSilverMineral1X(){return silverMineral1X;}
        public int getSilverMineral2X(){return silverMineral2X;}
        public double getAbsoluteConfidence() {return absoluteConfidence;}
        public int getItemCount(){return itemCount;}

        private GoldPosition getAbsolutePosition(int position){

            final int screenLeftRegionBorder = 333;
            final int screenRightRegionBorder = 767;
            GoldPosition calculatedPosition;

            if (position < screenLeftRegionBorder)
                calculatedPosition = GoldPosition.LEFT;
            else if (position >= screenRightRegionBorder)
                calculatedPosition = GoldPosition.RIGHT;
            else if(position >= screenLeftRegionBorder && position < screenRightRegionBorder)
                calculatedPosition=GoldPosition.CENTER;
            else calculatedPosition = GoldPosition.UNKNOWN;

            return calculatedPosition;
        }

        //And setters
        public void setItemCount (int numItems){
            itemCount=numItems;
        }

        public TensorFlowRefactor invoke() {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.

            // Not sure on actual method for determining view area size for X dimension
            // But empirically, objects .getLeft() method can return values anywhere between
            // -100 < X < 1200.  Set ranges based on this
            //  ||  LEFT    ||  CENTER      ||  RIGHT   ||
            //  ||  <333    || 333<=x<766  ||  >=767    ||

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {

                telemetry.addData("Item Count before purge", updatedRecognitions.size());
                if(updatedRecognitions.size()>3) {
                    Recognition currentRecognition;
                    int bottomThirdYLimit = 480;
                    for (Iterator<Recognition> iterator = updatedRecognitions.iterator(); iterator.hasNext(); ) {
                        currentRecognition = iterator.next();
                        if (currentRecognition.getBottom() < 480) {
                            iterator.remove();
                        }
                    }
                }
                telemetry.addData("Item Count after purge", updatedRecognitions.size());

                telemetry.update();
                //If at least 1 sampling item is visible, but not 4
                //Assign location X location of each mineral
                //IF 3 visible, assign relative position
                //Assign absolute position to gold mineral, or exclude positions if gold not seen
                setItemCount(updatedRecognitions.size());
                if (updatedRecognitions.size() > 0 && updatedRecognitions.size() < 4) {
                    for (Recognition recognition : updatedRecognitions) {
                        //  Assign X position ot each mineral identified
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    //If 3 sampling mineral positions determined, Use relative position to determine gold position
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        goldRelativePositionDetermined = true;
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            goldRelativePosition = GoldPosition.LEFT;
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            goldRelativePosition = GoldPosition.RIGHT;
                        } else {
                            goldRelativePosition = GoldPosition.CENTER;
                        }

                    }

                    //If the gold mineral is visible, determine absolute position
                    if (goldMineralX != -1) {
                        absoluteConfidence = 1;  //gold is positively identified
                        goldAbsolutePosition = getAbsolutePosition(goldMineralX);
                    //  Or if only 2 silver minerals found then infer gold position
                    } else if (updatedRecognitions.size() == 2
                            && silverMineral1X != -1 && silverMineral2X != -1) {
                        //  2 silver objects identified, but not gold
                        //  Determine the position of the 2 silver minerals and infer gold position
                        silver1AbsolutePosition = getAbsolutePosition(silverMineral1X);
                        silver2AbsolutePosition = getAbsolutePosition(silverMineral2X);

                        //If either silver is in the left position
                        if (silver1AbsolutePosition == GoldPosition.LEFT
                                | silver2AbsolutePosition == GoldPosition.LEFT) {
                            //And either one is on the right side, then cube must be center
                            if (silver1AbsolutePosition == GoldPosition.RIGHT
                                    | silver2AbsolutePosition == GoldPosition.RIGHT) {
                                goldAbsolutePosition = GoldPosition.CENTER;
                                absoluteConfidence = 1;
                            //Or if either is center, then cube must be right
                            } else if (silver1AbsolutePosition == GoldPosition.CENTER
                                    | silver2AbsolutePosition == GoldPosition.CENTER){
                                goldAbsolutePosition = GoldPosition.RIGHT;
                                absoluteConfidence = 1;
                            //  But if can't positively identify the silver positions, don't render a verdict
                            } else{
                                //Unable to positively exclude 2 positions
                                goldAbsolutePosition = GoldPosition.UNKNOWN;
                                absoluteConfidence = 0;
                            }

                        //Neither silver was identified in the left position
                        //So the other two must be identified as center and right, or gold position not positively excluded
                        //If silvers are located center and right, then gold is LEFT
                        } else if (silver1AbsolutePosition == GoldPosition.CENTER | silver2AbsolutePosition == GoldPosition.CENTER
                                    && silver1AbsolutePosition == GoldPosition.RIGHT | silver2AbsolutePosition == GoldPosition.RIGHT){
                            goldAbsolutePosition = GoldPosition.LEFT;
                            absoluteConfidence = 1;
                        } else {
                            //Unable to positively exclude 2 positions
                            goldAbsolutePosition = GoldPosition.UNKNOWN;
                            absoluteConfidence = 0;
                        }
                    //  If only 1 silver object is found
                    //  Then record the silver position as positively not being the position
                    } else if(updatedRecognitions.size() == 1
                            && (silverMineral1X != -1 | silverMineral2X != -1)){
                        if (silverMineral1X != -1) {
                            goldNotLocated = getAbsolutePosition(silverMineral1X);
                        }
                        else if(silverMineral2X != -1) {
                            goldNotLocated = getAbsolutePosition(silverMineral2X);
                        }
                    }
                }
//                telemetry.addData("# Object Detected", updatedRecognitions.size());
//                telemetry.addData("Gold Relative Position", goldRelativePosition.toString());
//                telemetry.addData("Gold Absolute Position", goldAbsolutePosition.toString());
//                telemetry.addData("Not Gold Position", goldNotLocated.toString());
//                telemetry.addData("Silver1 X", silverMineral1X);
//                telemetry.addData("Silver2 X", silverMineral2X);
//                telemetry.update();
            }
            return this;
        }
    }

    private class PerformDriveStepCustomEncoderControl {
        private ArrayList<DcMotor> motors;
        private int encoderClicksPerRevolution;
        private double spinBoostFactor;
        private double driveAngle;
        private double expectedTravelTime;
        private double spinPower;
        private double drivePower;
        private double requiredSpinDistance;
        private double[] driveValues;
        private int[] encoderTargetValues;
        private boolean driveStepPositionReached;
        private boolean driveStepTimedOut;
        private boolean driveStepOvershootDetected;

        public PerformDriveStepCustomEncoderControl(ArrayList<DcMotor> motors, int encoderClicksPerRevolution, double spinBoostFactor, double driveAngle, double expectedTravelTime, double spinPower, double drivePower, double requiredSpinDistance, double[] driveValues, int[] encoderTargetValues) {
            this.motors = motors;
            this.encoderClicksPerRevolution = encoderClicksPerRevolution;
            this.spinBoostFactor = spinBoostFactor;
            this.driveAngle = driveAngle;
            this.expectedTravelTime = expectedTravelTime;
            this.spinPower = spinPower;
            this.drivePower = drivePower;
            this.requiredSpinDistance = requiredSpinDistance;
            this.driveValues = driveValues;
            this.encoderTargetValues = encoderTargetValues;
        }

        public boolean isDriveStepPositionReached() {
            return driveStepPositionReached;
        }

        public boolean isDriveStepTimedOut() {
            return driveStepTimedOut;
        }

        public boolean isDriveStepOvershootDetected() {
            return driveStepOvershootDetected;
        }

        public PerformDriveStepCustomEncoderControl invoke() {
            //Perform the drives step
            //  Control the acceleration at the beginning to minimize slippage
            //  Track how far the robot has traveled
            //  Look for error conditions in the heading, which may indicate contact with other objects


            //Now get prepared to move
            //  But before that, look at where we are relative to target position
            //  Use the min relative position to determine the drive scale factor

            //TODO:  Include consideration for steep angles where 1 wheel may only spin 1 revolution
            int transientBuffer = encoderClicksPerRevolution * 1;  //Scale the drive values within this range
            zeroDriveMotorEncoders(motors);
            int[] currentMotorPositions = new int[4];
            getMotorPositions(currentMotorPositions, motors);

            //determine the drive motor scale factor
            //  this is a number between the minPowerScale and 1 that varies
            //  with the position within the transient range
            double minPowerScale = 0.5;
            //During initial acceleration, the driveScaleFactor = minPowerScale
            double driveScaleFactor = minPowerScale;
            double[] scaledDriveVector = new double[4];

            //TODO: Fix drive vector calculation

            calculateFieldOrientedDriveVector(driveAngle,Math.toRadians(currentHeading),drivePower,spinPower*spinBoostFactor, driveValues);

            //prior to scaling the drive vectors based to create soft beginning and end
            //Scale the drive vector so the max value is equal to maxDrive
            //  Might also need to But also stretch the lower end.  For instance, if something is 0.25, it should be at 25% of the range
            //  between minDrive and maxDrive

            double maxDrive = 1;  //Motor performance doesn't increase past this point
            double minDrive = 0.10;  //Stall condition
            //Now capture the max drive value from the array
            double maxValue = findMaxAbsValue(driveValues);

            //If any of the values exceed 1, then all drive values must be scaled
            //Divide all drive values by the max value to achieve a new max value of 1
            scaleDrive(maxDrive/maxValue, driveValues);


            for (int i=0; i<scaledDriveVector.length; i++){
                scaledDriveVector[i] = driveValues[i] * driveScaleFactor;
                if(scaledDriveVector[i]<minDrive) scaledDriveVector[i]=0;
                motors.get(i).setPower(scaledDriveVector[i]);
            }

            //Setup some controls to manage drive step exit conditions
            //Create variables to evaluate position accuracy to target
            int targetPositionAccuracy = 50;
            driveStepPositionReached = false;
            int[] motorIdsForPositionTracking = new int[2];  //Gets the motor IDs for 2 motors that move the most
            getMotorIdsForTwoHighestMotors(encoderTargetValues,motorIdsForPositionTracking);

            //Create variables to capture timeout
            long driveStepEndTime = (System.nanoTime() / 1000000);  //current time in milliseconds
            long driveStepTimeBuffer = 3000;  //Buffer time in milliseconds for transient
            driveStepEndTime = driveStepEndTime + (long)(expectedTravelTime*1000) + driveStepTimeBuffer;
            //TODO:  REMOVE THIS AT SOME POINT (This doubles drive time from calc)
            //driveStepEndTime += (long)(expectedTravelTime*1000) + driveStepTimeBuffer;
            driveStepTimedOut = false;
            //Create variables to capture overshoot condition
            int consecutiveCyclesWithIncreasingError = 0;
            int previousPositionError = getNumClicksFromTarget(currentMotorPositions,encoderTargetValues,motorIdsForPositionTracking);
            int overshootCycleCountLimit = 7;
            driveStepOvershootDetected = false;

            int currentPositionError;
            int maxClicksFromStart;
            int positionForScaleFactor;
            //TODO:  Prioritize wheels for encoder control (pick top 2 spinners)
            //TODO:  Handle checks for negative numbers in ramp?  (Why did it jump when driving backwards)
            while (opModeIsActive() && !driveStepPositionReached  //If encoder is within this many clicks
                    & !driveStepTimedOut  //Or it takes too much time
                    //& !driveStepOvershootDetected
                    ){  //Or going to wrong way

                //Get current positions
                getMotorPositions(currentMotorPositions, motors);
                currentPositionError = getNumClicksFromTarget(currentMotorPositions, encoderTargetValues, motorIdsForPositionTracking);
                //Determine scale factor
                maxClicksFromStart = findMaxAbsValue(currentMotorPositions);
                positionForScaleFactor = (maxClicksFromStart < currentPositionError) ? maxClicksFromStart : currentPositionError;

                //See if position error has decreased from last cycle
                if(currentPositionError > previousPositionError){
                    consecutiveCyclesWithIncreasingError ++;
                    if (consecutiveCyclesWithIncreasingError > overshootCycleCountLimit) driveStepOvershootDetected = true;
                }else{
                    consecutiveCyclesWithIncreasingError=0;
                }

                previousPositionError = currentPositionError;

                if (positionForScaleFactor < transientBuffer) {
                    driveScaleFactor = ((1 - minPowerScale) / transientBuffer) * (transientBuffer - positionForScaleFactor);
                    driveScaleFactor = 1-driveScaleFactor;
                } else {
                    driveScaleFactor = 1;
                }

                //Calculate new drive vector and set motor power
                getCurrentHeading();
                calculateFieldOrientedDriveVector(driveAngle, Math.toRadians(currentHeading), drivePower, spinPower*spinBoostFactor, driveValues);
                maxValue = findMaxAbsValue(driveValues);

                //If any of the values exceed 1, then all drive values must be scaled
                //Divide all drive values by the max value to achieve a new max value of 1
                scaleDrive(maxDrive/maxValue, driveValues);


                for (int i=0; i<scaledDriveVector.length; i++){
                    scaledDriveVector[i] = driveValues[i] * driveScaleFactor;
                    if(scaledDriveVector[i]<minDrive) scaledDriveVector[i]=0;
                    motors.get(i).setPower(scaledDriveVector[i]);
                }


                //Check if position reached or step has timed out
                if (currentPositionError < targetPositionAccuracy) driveStepPositionReached = true;
                if ((System.nanoTime() / 1000000) > driveStepEndTime) driveStepTimedOut = true;

                telemetry.addData("Calculated Travel Time", expectedTravelTime);
                telemetry.addData("Spin Power", spinPower);
                telemetry.addData("drivePower", drivePower);
                telemetry.addData("driveAngle", driveAngle);
                telemetry.addData("spinDistance", requiredSpinDistance);
                telemetry.addData("Heading", currentHeading);
                telemetry.addData("radHeading", radHeading);
                telemetry.addData("target value0", Arrays.toString(encoderTargetValues));
                telemetry.addData("Actual Value0", Arrays.toString(currentMotorPositions));
                telemetry.update();
            }

            //Stop all motors
            stopMotors(motors);
            return this;
        }

        private void zeroDriveMotorEncoders(ArrayList<DcMotor> motors) {
            for (DcMotor m:motors){
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        private void getMotorIdsForTwoHighestMotors(int[] encoderTargetValues, int[] returnMotorIds){
            int primaryMotor = 0;
            int primaryValue = encoderTargetValues[0];
            int secondaryMotor=0;
            int secondaryValue = 0;
            for(int i=1;i<encoderTargetValues.length; i++){
                if(Math.abs(encoderTargetValues[i])>=primaryValue){
                    //The primary value will be erased, but first see if the secondary should be erased
                    if(primaryValue >= secondaryValue){
                        secondaryValue = primaryValue;
                        secondaryMotor = primaryMotor;
                    }
                    primaryMotor=i;
                    primaryValue = Math.abs(encoderTargetValues[i]);
                }
            }
            returnMotorIds[0] = primaryMotor;
            returnMotorIds[1] = secondaryMotor;
        }

        private int getNumClicksFromTarget(int[] currentMotorPositions, int[] encoderTargetValues, int[] consideredMotorIds) {
            //returns the minimum error from the target encoder value

    //        int[] clicksFromTarget = new int[4];
    //        int dummyClickValue = 5000;
    //        for(int i=0; i<currentMotorPositions.length; i++) {
    //            if (encoderTargetValues[i] == 0) {
    //                //If that motor is not meant to be driven, use a dummy value
    //                clicksFromTarget[i] = dummyClickValue;
    //            } else {
    //                clicksFromTarget[i] = encoderTargetValues[i] - currentMotorPositions[i];
    //            }
    //        }
            //Note, there are only 2 motors that are being considered for position accuracy
            //Look at the first motor position vs its target
            int motorID = consideredMotorIds[0];
            int clicksFromTarget = Math.abs(currentMotorPositions[motorID] - encoderTargetValues[motorID]);
            //Now switch to the next motor
            motorID = consideredMotorIds[1];
            //If the second motor has less positional error, use that value
            if(clicksFromTarget > Math.abs(currentMotorPositions[motorID] - encoderTargetValues[motorID])){
                clicksFromTarget = Math.abs(currentMotorPositions[motorID] - encoderTargetValues[motorID]);
            }

            return clicksFromTarget;
        }
    }
}
