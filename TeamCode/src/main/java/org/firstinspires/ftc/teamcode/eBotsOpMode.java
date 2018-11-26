package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

import java.sql.Array;
import java.util.ArrayList;
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
    public Orientation angles;             //For imu gyroscope
    public Acceleration gravity;           // State used for updating telemetry


    //Variables for the TensorFlow Object Detection
    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    public final static String VUFORIA_KEY = "AdGgXjv/////AAABmSSQR7vFmE3cjN2PqTebidhZFI8eL1qz4JblkX3JPyyYFRNp/Su1RHcHvkTzJ1YjafcDYsT0l6b/2U/fEZObIq8Si3JYDie2PfMRfdbx1+U0supMRZFrkcdize8JSaxMeOdtholJ+hUZN+C4Ovo7Eiy/1sBrqihv+NGt1bd2/fXwvlIDJFm5lJHF6FCj9f4I7FtIAB0MuhdTSu4QwYB84m3Vkx9iibTUB3L2nLLtRYcbVpoiqvlxvZomUd2JMef+Ux6+3FA3cPKCicVfP2psbjZrxywoc8iYUAq0jtsEaxgFdYoaTR+TWwNtKwJS6kwCgBWThcIQ6yI1jWEdrJYYFmHXJG/Rf/Nw8twEVh8l/Z0M";

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

    public double getCurrentHeading(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = angles.firstAngle;
        return currentHeading;
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
    public void twistToAngle(double spinAngle, double speed, ArrayList<DcMotor> motors){
        //Note this logic is demoed in the "Gyro" tab of "Rover Time Trials" Google Sheets file
        boolean currentHeadingModifier = false;  //Used as a flag if the target angle passes the 180 point
        double overFlowAngle = 0;
        double adjustedAngle = getCurrentHeading();  //Adjusted angle is to handle crossover of sign at 180 degrees
        double angleBufferForPrecision = 60;
        double fullSpeed = 0.5;
        double throttleDownSpeed = 0.15;
        double slopeForThrottleDown = (fullSpeed-throttleDownSpeed)/angleBufferForPrecision;

        //Create loop so robot spins until target angle is achieved
        //Based on the field coordinate system, positive roll is spinning to the left
        //But the drive vector equations consider turning to the right to be positive
        //So to establish a targetAngle, the desired spinAngle must be subtracted from the currentHeading
        double targetAngle = adjustedAngle - spinAngle;

        //The imu changes sign at 180 degrees
        // For this control loop to work, must catch this CROSSOVER event
        if (Math.abs(targetAngle)>180){
            currentHeadingModifier = true;
        }
        double angleFromTarget = Math.abs(spinAngle);
        if(angleFromTarget < angleBufferForPrecision){
            //This formula was checked on Google Sheets
            speed = fullSpeed - ((slopeForThrottleDown) * (angleBufferForPrecision-angleFromTarget));
        }
        startSpinning(spinAngle, speed, motors);

        //TODO: Compress these two loops into 1
        if(spinAngle > 0){
            //If spinning to the right, keep spinning while angle is greater than target angle
            // When spinning right, imu angle is decreasing
            while(opModeIsActive() && adjustedAngle > targetAngle) {
                //  Just keep spinning to the right

                //Get the new adjusted angle
                adjustedAngle = getCurrentHeading();
                if (currentHeadingModifier && adjustedAngle > 120) {
                    //This detects when crossover occurs, must add overflow
                    //And applies the adjustment to the current heading if necessary
                    overFlowAngle = 180 - adjustedAngle;
                    adjustedAngle = -180 - overFlowAngle;
                }

                //If close to the target angle
                angleFromTarget = Math.abs(adjustedAngle - targetAngle);
                if (angleFromTarget < angleBufferForPrecision){
                    speed = fullSpeed - ((slopeForThrottleDown) * (angleBufferForPrecision-angleFromTarget));
                    startSpinning(spinAngle,speed, motors);
                }

                telemetry.addData("Target Spin", spinAngle);
                telemetry.addData("Target Angle", targetAngle);
                telemetry.addData("currentHeading", adjustedAngle);
                telemetry.addData("Status", "Turning");
                telemetry.update();
            }
        }else{
            //If spinning to the left, keep spinning while angle is less than target angle
            //When spinning left, the imu angle is increasing
            //so keep spinning while heading is less than target
            while (opModeIsActive() && adjustedAngle < targetAngle) {
                //  Just keep spinning to the left

                //Get the new adjusted angle
                adjustedAngle = getCurrentHeading();

                if(currentHeadingModifier && adjustedAngle < -120) {
                    //If crossover occurs
                    //And applies the adjustment to the current heading if necessary

                    overFlowAngle = 180 + adjustedAngle;  //add current heading since large negative
                    adjustedAngle = 180 + overFlowAngle;
                }

                angleFromTarget = Math.abs(adjustedAngle - targetAngle);
                if (Math.abs(adjustedAngle - targetAngle) < angleBufferForPrecision){
                    speed = fullSpeed - ((slopeForThrottleDown) * (angleBufferForPrecision-angleFromTarget));
                    startSpinning(spinAngle,speed, motors);
                }

                telemetry.addData("Target Spin", spinAngle);
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

        long stopTime = 500;
        long currentTime = System.nanoTime() / 1000000;
        for (long t = currentTime; t < (currentTime + stopTime); t = (System.nanoTime() / 1000000)) {
            for (DcMotor m : motors) {
                m.setPower(0);
            }
        }
    }

    private boolean turnToFieldHeading (double desiredFieldHeading, ArrayList<DcMotor> motors){
        // Field heading is the imu direction based on the assumed zero point from lander
        // This command calculates the turn inputs for twistToAngle function and calls it
        double requiredTurnAngle = checkHeadingVersusTarget(desiredFieldHeading);  //How many angles must turn
        twistToAngle(requiredTurnAngle,0.35, motors);
        double turnError = checkHeadingVersusTarget(desiredFieldHeading);

        if(Math.abs(turnError)<5){
            return true;
        }else{
            return false;
        }
    }

    public boolean moveByDistance(double inchesForward, double inchesLateral, double rotationAngle, ArrayList<DcMotor> motors){
        //  For this method, forward travel is relative to robot heading at start of move
        //  For spin, positive spin is towards right, negative spin is towards left
        //  Wheel orientation assumes X orientation when looking from above down to ground plane


        //Analyze the move and determine how much distance each wheel must travel
        //  Calculate the drive power vector for the motors
        //  Determine the target heading
        //  Determine time to complete

        //Initialize variables for drive distance calculations
        int encoderClicksPerRevolution = R.integer.NEVEREST_20_CPR;
        double wheelDiameter = 3.0;
        double wheelDiagonalSpacing = 21.3;
        double peakRobotSpeed = 30;  //peak robot speed in inches per second

        double inchesPerRevolution = wheelDiameter * Math.PI;
        int encoderClicksPerInch = (int) Math.round(encoderClicksPerRevolution / inchesPerRevolution);

        double spinCircumference = wheelDiagonalSpacing * Math.PI;
        double requiredSpinDistance = rotationAngle/360 * spinCircumference;  //distance in inches required for spin

        //Calculate wheel revolution distance for travel
        double robotAngle = Math.atan2(inchesForward, inchesLateral);   //  Angle used for mecanum drive vector
        double travelDistance = Math.sqrt(Math.pow(inchesForward,2) + Math.pow(inchesLateral,2));

        //Set the target heading.  Note that rotationAngle is positive if to right, which is opposite imu direction
        //That is why rotationAngle is subtracted from currentHeading
        //Also note that the target heading can have abs value greater than 180, which may be a problem later
        currentHeading = getCurrentHeading();  //Refresh the current heading
        double targetHeading = currentHeading-rotationAngle;

        //Deal with angle overflow (abs(angle)>180)
        if (targetHeading > 180){
            targetHeading = -180 - (180-targetHeading);  //subtracting a negative number from -180 reduces magnitude
        }else if (targetHeading < -180){
            targetHeading = 180-(-180-targetHeading);  //note targetHeading is negative and larger magnitude, resulting in positive number
        }

        //Note, drive values assume the following order of wheels  [FL, FR, BL, BR]
        //  This calculation is used to determine distances, and isn't the final calc for motors
        double[] driveValues = new double[4];
        calculateDriveVector(1,robotAngle,0,driveValues);

        //Now using the drive values, drive distances can be calculated
        //This is based on the principle that the wheel rotation is generally greater than straight line
        //For instance, if traveling forward, the drive value is 0.707
        //and the drive distance for 10in travel is 10/0.707 = 14.1
        double[] driveDistances = new double[4];
        int[] encoderTargetValues = new int[4];
        for (int i=0; i<4; i++){
            driveDistances[i] = travelDistance/driveValues[i];

            //Now, if the robot is spinning, must add the spin distance
            //Based on wheel configuration, spin is added respectively [FL, FR, BL, FR] --> [1, -1, 1, -1]
            int motorRotationSign =  (i%2 == 0) ? 1 : -1;  //Add for entries 0 and 2, subtract for 1 and 3
            driveDistances[i] = driveDistances[i]+(requiredSpinDistance*motorRotationSign);

            //Convert the distances to encoder targets
            //TODO:  Verify the signs for each motor based on spin direction
            encoderTargetValues[i] = (int) driveDistances[i] * encoderClicksPerInch;
        }

        //  Find the max distance traveled and determine expected travel time
        double maxDistance = findMaxAbsValue(driveDistances);
        double expectedTravelTime = maxDistance / peakRobotSpeed;

        //Now that the distances are know for each wheel, determine drive vectors
        //  Assume that the motor speed is linearly proportional from 0-0.85
        double maxTargetDriveValue = 0.85;  //Max motor power
        double minPowerThreshold = 0.1;     //Min motor power
        double distanceDriveScaleFactor = maxTargetDriveValue / maxDistance;
        if (maxDistance > maxTargetDriveValue) scaleDrive(distanceDriveScaleFactor, driveDistances);  //Scale drive
        //If drive power too low, zero it out and also zero the corresponding encoder target
        //Note that after scaling, driveDistances is the vector for motor power
        for (int i=0; i<driveDistances.length; i++) {
            if(driveDistances[i]<minPowerThreshold) {
                driveDistances[i]=0;
                encoderTargetValues[i] = 0;
            }
        }

        //Perform the drive
        //  Control the acceleration at the beginning
        //  Track how far the robot has traveled
        //  Look for error conditions in the heading, which may indicate contact with other objects


        //Now get prepared to move
        //  But before that, look at where we are relative to target position
        //  Use the min relative position to determine the drive scale factor
        int transientBuffer = encoderClicksPerRevolution * 2;  //Scale the drive values over this range
        boolean inTransientSpeed = true;
        zeroDriveMotorEncoders(motors);
        int[] currentMotorPositions = new int[4];
        getMotorPositions(currentMotorPositions, motors);
        int maxClicksFromStart = findMaxAbsValue(currentMotorPositions);
        int minClicksFromTarget = getNumClicksFromTarget(currentMotorPositions,encoderTargetValues);
        int positionForScaleFactor = (maxClicksFromStart<minClicksFromTarget) ? maxClicksFromStart : minClicksFromTarget;

        //determine the drive motor scale factor
        //  this is a number between the minPowerScale and 1 that varies
        //  with the position within the transient range
        double minPowerScale = 0.3;
        double driveScaleFactor = ((1-minPowerScale)/transientBuffer) * (transientBuffer-positionForScaleFactor);
        double[] scaledDriveVector = new double[4];
        for (int i=0; i<scaledDriveVector.length; i++){
            scaledDriveVector[i] = driveDistances[i] * driveScaleFactor;
            motors.get(i).setPower(scaledDriveVector[i]);
        }

        //Setup some controls to manage the amount of time taken during a drive step
        long driveStepEndTime = (System.nanoTime() / 1000000);  //current time in milliseconds
        long driveStepTimeBuffer = 3500;  //Buffer time in milliseconds for transient
        driveStepEndTime = driveStepEndTime + (long)(expectedTravelTime*1000) + driveStepTimeBuffer;

        while (minClicksFromTarget > 50  //If encoder is within this many clicks
                & (System.nanoTime() / 1000000) < driveStepEndTime){  //Or it takes too much time {
            //Get current positions
            getMotorPositions(currentMotorPositions, motors);

            //Determine scale factor
            maxClicksFromStart = findMaxAbsValue(currentMotorPositions);
            minClicksFromTarget = getNumClicksFromTarget(currentMotorPositions, encoderTargetValues);
            positionForScaleFactor = (maxClicksFromStart < minClicksFromTarget) ? maxClicksFromStart : minClicksFromTarget;

            if (positionForScaleFactor < transientBuffer) {
                inTransientSpeed = true;
                driveScaleFactor = ((1 - minPowerScale) / transientBuffer) * (transientBuffer - positionForScaleFactor);
            } else {
                inTransientSpeed = false;
                driveScaleFactor = 1;
            }

            //Calculate new drive vector and set motor power
            for (int i = 0; i < scaledDriveVector.length; i++) {
                scaledDriveVector[i] = driveDistances[i] * driveScaleFactor;
                motors.get(i).setPower(scaledDriveVector[i]);
            }
        }

        //Check the heading at the end of the move and correct it if necessary
        double headingError = checkHeadingVersusTarget(targetHeading);

        //If error is too high, correct
        if (Math.abs(headingError)>10)  {
            twistToAngle(headingError,0.2,motors);
        }

        if (Math.abs(headingError) < 5){
            return true;
        }else{
            return false;
        }

    }

    private double checkHeadingVersusTarget(double targetHeading){
        //Check the heading at the end of the move and correct it if necessary
        getCurrentHeading();
        double headingError = currentHeading-targetHeading;
        //If large error, assume that crossover has occurred
        if (Math.abs(headingError)>180){
            //Correct the heading error caused by crossover
            //This math was tested in Google Sheets
            headingError = (360 - Math.abs(headingError)) * Math.signum(targetHeading);
        }
        return headingError;
    }

    private int getNumClicksFromTarget(int[] currentMotorPositions, int[] encoderTargetValues) {
        int[] clicksFromTarget = new int[4];
        int dummyClickValue = 5000;
        for(int i=0; i<currentMotorPositions.length; i++) {
            if (encoderTargetValues[i] == 0) {
                //If that motor is not meant to be driven, use a dummy value
                clicksFromTarget[i] = dummyClickValue;
            } else {
                clicksFromTarget[i] = encoderTargetValues[i] - currentMotorPositions[i];
            }
        }
        return findMinAbsValue(clicksFromTarget);
    }

    private void getMotorPositions(int[] currentPositions, ArrayList<DcMotor> motors) {
        for(int i=0; i<currentPositions.length; i++){
            currentPositions[i] = motors.get(i).getCurrentPosition();
        }
    }

    private void zeroDriveMotorEncoders(ArrayList<DcMotor> motors) {
        for (DcMotor m:motors){
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
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

                //If at least 1 sampling item is visible, but not 4
                //Assign location X location of each mineral
                //IF 3 visible, assign relative position
                //Assign absolute position to gold mineral, or exclude positions if gold not seen
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
}
