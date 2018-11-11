package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
//import java.lang.String;

@Autonomous
public class Auton_Crater extends LinearOpMode {

    //This autonomous mode performs a sequence of tasks intended to :
    // move rover from the depot latch on the lander
    // sample
    // claim depot
    // park in crater
    // detailed list of operations is listed in opMode main body


    //private Gyroscope imu;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    //Manipulation Motors
    private DcMotor armAngleMotor;
    private DcMotor armExtensionMotor;
    private DcMotor collectMotor;
    private DcMotor latchMotor;

    private BNO055IMU imu;          //Create a variable for the gyroscope on the Expansion Hub
    private double currentHeading;  //Angular direction in Degrees
    Orientation angles;             //For imu gyroscope
    Acceleration gravity;           // State used for updating telemetry


    //Variables for the TensorFlow Object Detection
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    //  DEFINE CONSTANTS FOR THE ROBOT
    final static int ARM_ANGLE_COLLECT_POSITION = -13400;  //TEST POSITIION
    final static int ARM_ANGLE_TRAVEL_POSITION = -5600;
    final static int ARM_ANGLE_DUMP_POSITION = -12500;

    //These are constants used to define counts per revolution of NEVEREST motors with encoders
    static final int NEVEREST_60_CPR = 1680;
    static final int NEVEREST_40_CPR = 1120;
    static final int NEVEREST_20_CPR = 560;

    //final int ARM_EXTENSION_COLLECTION_POSITION = 27800;
    final static int ARM_EXTENSION_COLLECTION_POSITION = -57500;  //test position
    final static int ARM_EXTENSION_TRAVEL_POSITIION = -27800;
    final static int ARM_EXTENSION_DUMP_POSITION = -27800;

    final static int LATCH_DEPLOY_POSITION = 13200;        //13900 is a little too high
    //13500 is a little high for our practice lander
    final static int LATCH_DRIVE_POSITION = 5900;          //5900 is good, could be a little higher
    final static int LATCH_ENGAGE_POSITION = 9892;
    final static int LATCH_RISEUP_POSITION = 1540;

    //MOTOR PROTECTION ENCODER
    final static int ARM_ANGLE_LIMIT = -13000;
    final static int ARM_EXTENSION_LIMIT = 27850;
    final static int LATCH_LIMIT = 14000;

    final static String VUFORIA_KEY = "AdGgXjv/////AAABmSSQR7vFmE3cjN2PqTebidhZFI8eL1qz4JblkX3JPyyYFRNp/Su1RHcHvkTzJ1YjafcDYsT0l6b/2U/fEZObIq8Si3JYDie2PfMRfdbx1+U0supMRZFrkcdize8JSaxMeOdtholJ+hUZN+C4Ovo7Eiy/1sBrqihv+NGt1bd2/fXwvlIDJFm5lJHF6FCj9f4I7FtIAB0MuhdTSu4QwYB84m3Vkx9iibTUB3L2nLLtRYcbVpoiqvlxvZomUd2JMef+Ux6+3FA3cPKCicVfP2psbjZrxywoc8iYUAq0jtsEaxgFdYoaTR+TWwNtKwJS6kwCgBWThcIQ6yI1jWEdrJYYFmHXJG/Rf/Nw8twEVh8l/Z0M";


    final static long SAMPLING_DRIVE_TIME = 1400;
    final static long SAMPLING_EXTRA_DRIVE_TIME = 300;
    final static double SAMPLING_DRIVE_COMPONENT = 0.70;

    final static long DEPOT_DRIVE_TIME = 1100;
    final static long DEPOT_EXTRA_DRIVE_TIME = 0;
    final static double DEPOT_DRIVE_COMPONENT = .4;
    final static long CRATER_DRIVE_TIME = 3500;  //was 2500 for first successful sample

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration for gyro
    //----------------------------------------------------------------------------------------------
    void composeTelemetry() {
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

    private void calculateDriveVector(double driveMagnitude, double robotAngle, double spin, double[] outputArray){
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

    private void scaleDrive (double scaleFactor, double[] driveArray){
        for (int i=0; i<driveArray.length; i++) {
            driveArray[i] *= scaleFactor;
        }
    }

    private double findMaxAbsValue(double[] array) {
        //Now capture the max drive value from the array
        double maxValue = Math.abs(array[0]);
        for (int i = 1; i < array.length; i++){
            if(Math.abs(array[i]) > maxValue) maxValue = Math.abs(array[i]);
        }
        return maxValue;
    }

    private void performDriveStep(double xInput, double yInput, double spin, long duration, ArrayList<DcMotor> motors){
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

    private void twistToAngle(double spinAngle, double speed,  ArrayList<DcMotor> motors){
        //Note this logic is demoed in the "Gyro" tab of "Rover Time Trials" Google Sheets file
        boolean currentHeadingModifier = false;  //Used as a flag if the target angle passes the 180 point
        boolean startingSignPositive;           //Used to know starting sign of angle
        double overFlowAngle = 0;
        double adjustedAngle;
        startSpinning(spinAngle, speed, motors);

        //Create loop so robot spins until target angle is achieved
        //Based on the field coordinate system, positive roll is spinning to the left
        //But the drive vector equations consider turning to the right to be positive
        //So to establish a targetAngle, the desired spinAngle must be subtracted from the currentHeading
        double targetAngle = currentHeading - spinAngle;
        if (Math.abs(targetAngle)>180){
            currentHeadingModifier = true;
            if (currentHeading < 0) {
                startingSignPositive = false;
            } else {
                startingSignPositive = true;
            }
        }
        if(spinAngle > 0){
            if(!currentHeadingModifier){
            //If spinning to the right, keep spinning while angle is greater than target angle
            while(currentHeading > targetAngle){
                //  Just keep spinning to the right
                telemetry.addData("Target Spin", spinAngle);
                telemetry.addData("Target Angle", targetAngle);
                telemetry.addData("currentHeading", currentHeading);
                telemetry.addData("Status", "Turning");
                telemetry.update();
            }} else {
                adjustedAngle = currentHeading;
                while(adjustedAngle > targetAngle){
                    if(currentHeading > 120){
                        adjustedAngle = -180-(180-currentHeading);
                    }
                    telemetry.addData("Target Spin", spinAngle);
                    telemetry.addData("Target Angle", targetAngle);
                    telemetry.addData("currentHeading", adjustedAngle);
                    telemetry.addData("Status", "Turning");
                    telemetry.update();
                }
            }
        }else{
            //If spinning to the left, keep spinning while angle is less than target angle
            while(currentHeading < targetAngle){
                //  Just keep spinning to the left
                telemetry.addData("Target Spin", spinAngle);
                telemetry.addData("Target Angle", targetAngle);
                telemetry.addData("currentHeading", currentHeading);
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

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }


    @Override
    public void runOpMode(){
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
        ArrayList<DcMotor> motorList= new ArrayList<>();
        motorList.add(frontLeft);
        motorList.add(frontRight);
        motorList.add(backLeft);
        motorList.add(backRight);

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
        latchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        latchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Initialize the variables that are being used in the main loop
        double spinAngle;    //NOTE:  Positive spin is towards right
        double driveX;
        double driveY;
        double spinSpeed;
        long delayTime;  //delay time in milliseconds
        long extraDelayTime;  //Based on drive trajectory, sometimes extra time is needed
        double driveComponentForSamplingDirection;
        boolean goldPositionDetermined = false;
        String goldPosition = "";
        //Wait for the game to start(driver presses PLAY)

        // Set up our telemetry dashboard
        composeTelemetry();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }


        waitForStart();
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        //This autonomous mode performs the following tasks:
        //  1)  a) Scans from latch
        //      a) Lands
        //      b) move away from lander a little bit
        //  2)  Scans the sampling area to try to identify the cube location
        //  3)  Moves away from the lander
        //  4)  Lowers the latch to prepare for reattaching
        //  5)  a) Twist 95 degrees to sample object
        //      b) Twist back 5 degrees to orient towards depot
        //  6)  Move to Depot
        //  7)  Lower arm angle to drop marker
        //  8) Retract arm angle to drive position
        //  9) Turn towards the crater
        //  10) Move to the crater
        //  11) Extend the Collector Arm and lower the arm angle

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  1) Land -->Unlatch from lander

        if (tfod != null) {
            tfod.activate();
        }

        latchMotor.setTargetPosition(LATCH_DEPLOY_POSITION);
        latchMotor.setPower(1);

        while(latchMotor.isBusy()){
            //Don't do anything until the latch is done moving
            //This is where the vuforia code should go for the camera
            if (tfod != null && !goldPositionDetermined) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    //If all three sampling items are visible, then try to identify the position of the gold
                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            goldPositionDetermined = true;
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                goldPosition = "Left";
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                goldPosition = "Right";
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                goldPosition = "Center";
                            }
                        }
                    } else if(updatedRecognitions.size() == 2 & !goldPositionDetermined) {
                        //If haven't determined position from the 3 objects
                        //And only see 2 objects
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }

                        //Infer where gold is if only 2 objects recognized
                        //Assume we only see the RIGHT two
                        if (goldMineralX == -1 && silverMineral1X != -1){
                            //If gold isn't one of the two objects
                            //And assuming we only see the left 2 objects
                            goldPosition = "Right";
                            telemetry.addData("Gold Mineral Position", goldPosition);
                        }  else if (goldMineralX != -1 && silverMineral1X != -1){
                            if (goldMineralX < silverMineral1X){
                                goldPosition = "Left";
                                telemetry.addData("Gold Mineral Position", goldPosition);
                            } else {
                                goldPosition = "Center";
                                telemetry.addData("Gold Mineral Position", goldPosition);
                            }
                        }

                    }
                    telemetry.update();
                }
            }
            telemetry.update();
        }
        latchMotor.setPower(0);

        //Now, if the goldPosition was set based on 2 objects being observed, then use that value
        if(goldPosition != "") goldPositionDetermined = true;

        //if cube location is not determined, then randomly assign a location
        if(!goldPositionDetermined){
            if(Math.random() <= 0.5){
                goldPosition = "Left";
            } else {
                goldPosition = "Right";
            }
        }
        telemetry.addData("Gold Mineral Position", goldPosition);
        telemetry.update();


        //  b) move away from the lander a little bit
        //Delay a small amount while moving
        delayTime = 300;
        driveX = 1;
        driveY = 0;
        spinSpeed = 0;
        performDriveStep(driveX, driveY, spinSpeed, delayTime, motorList);  //Just a pause, no movement


        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  3) Moves away from the lander
        //  Toward sample, note that it is moving sideways

        driveX = 1;         //Motion
        delayTime = SAMPLING_DRIVE_TIME;
        extraDelayTime = SAMPLING_EXTRA_DRIVE_TIME;
        driveComponentForSamplingDirection = SAMPLING_DRIVE_COMPONENT;

        //************************************
        //Move to sample
        //    Based on the position of the gold element, align to knock it off
        if (goldPosition == "Left"){
            driveY = driveComponentForSamplingDirection;         //Motion in y direction to get to left spot
            delayTime += extraDelayTime;        //Add extra drive time because longer distance to spot
        } else if (goldPosition == "Right"){
            driveY = -driveComponentForSamplingDirection;
            delayTime += extraDelayTime;  //Add extra drive time because longer distance to spot
        } else {
            driveY = 0;
        }
        spinSpeed = 0;      //This is used to determine how to spin the robot
        //This function will perform the drive step and stop the motors
        performDriveStep(driveX, driveY, spinSpeed, delayTime, motorList);


        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  4) Lowers the latch to prepare for reattaching
        //      After starting the drive step, move latch down to the DRIVE_POSITION
        latchMotor.setTargetPosition(LATCH_DRIVE_POSITION);
        latchMotor.setPower(1);


        //  6)  Move to Depot
        //      Based on where the sampling occurred, the depot drive direction must be determined
        driveX = 1;         //Toward robot front
        delayTime = DEPOT_DRIVE_TIME;
        extraDelayTime = DEPOT_EXTRA_DRIVE_TIME;       //Extra time required for extra distance based on sampling location
        driveComponentForSamplingDirection = DEPOT_DRIVE_COMPONENT;

        //    Based on the position of the gold element, align to knock it off
        if (goldPosition == "Left"){
            driveY = -driveComponentForSamplingDirection;         //Motion in y direction to get to left spot
            delayTime += extraDelayTime;        //Add extra drive time because longer distance to spot
        } else if (goldPosition == "Right"){
            driveY = +driveComponentForSamplingDirection;
            delayTime += extraDelayTime;  //Add extra drive time because longer distance to spot
        } else {
            driveY = 0;
        }
        spinSpeed = 0;      //This is used to determine how to spin the robot
        //This function will perform the drive step and stop the motors
        performDriveStep(driveX, driveY, spinSpeed, delayTime, motorList);
        //  7)  Lower arm angle to drop marker
        //armAngleMotor.setTargetPosition(ARM_ANGLE_DUMP_POSITION);
        armAngleMotor.setTargetPosition(ARM_ANGLE_TRAVEL_POSITION);
        armAngleMotor.setPower(1);

        armExtensionMotor.setTargetPosition(ARM_EXTENSION_COLLECTION_POSITION);
        armExtensionMotor.setPower(1);

        while (armAngleMotor.isBusy() | armExtensionMotor.isBusy()){
            //Wait for it to complete
        }
    }
}
