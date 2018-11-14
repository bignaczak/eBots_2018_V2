package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

//@Disabled
@Autonomous
public class Auton_Depot extends LinearOpMode {

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
    //  THESE ARE ALL POSITIONS ASSUMED THE ROBOT IS COMPLETELY FOLDED PRIOR TO START OF AUTON
    final static int ARM_ANGLE_COLLECT_POSITION = -11750;  //REAL POSITION
    //final static int ARM_ANGLE_COLLECT_POSITION = -10000;  //TEST POSITION
    final static int ARM_ANGLE_TRAVEL_POSITION = -5600;
    final static int ARM_ANGLE_SCORE_POSITION = -1000;    //MUST VERIFY
    final static int ARM_ANGLE_DUMP_POSITION = -9500;      //-11000 might be too low, can hit glass

    //These are constants used to define counts per revolution of NEVEREST motors with encoders
    static final int NEVEREST_60_CPR = 1680;
    static final int NEVEREST_40_CPR = 1120;
    static final int NEVEREST_20_CPR = 560;

    //final int ARM_EXTENSION_COLLECTION_POSITION = 27800;
    final static int ARM_EXTENSION_COLLECTION_POSITION = -57500;  //test position
    final static int ARM_EXTENSION_TRAVEL_POSITIION = -27800;
    final static int ARM_EXTENSION_DUMP_POSITION = -27800;

    final static int LATCH_DEPLOY_POSITION = -13200;        //13900 is a little too high
    //13500 is a little high for our practice lander
    final static int LATCH_DRIVE_POSITION = -5900;          //5900 is good, could be a little higher
    final static int LATCH_ENGAGE_POSITION = -11000;
    final static int LATCH_RISEUP_POSITION = -2500;

    //MOTOR PROTECTION ENCODER
    final static int ARM_ANGLE_LIMIT = -11750;  //REAL POSITION
    //final static int ARM_ANGLE_LIMIT = -10000;  //TEST POSITION
    final static int ARM_EXTENSION_LIMIT = -57500;
    final static int LATCH_LIMIT = -14000;

    final static String VUFORIA_KEY = "AdGgXjv/////AAABmSSQR7vFmE3cjN2PqTebidhZFI8eL1qz4JblkX3JPyyYFRNp/Su1RHcHvkTzJ1YjafcDYsT0l6b/2U/fEZObIq8Si3JYDie2PfMRfdbx1+U0supMRZFrkcdize8JSaxMeOdtholJ+hUZN+C4Ovo7Eiy/1sBrqihv+NGt1bd2/fXwvlIDJFm5lJHF6FCj9f4I7FtIAB0MuhdTSu4QwYB84m3Vkx9iibTUB3L2nLLtRYcbVpoiqvlxvZomUd2JMef+Ux6+3FA3cPKCicVfP2psbjZrxywoc8iYUAq0jtsEaxgFdYoaTR+TWwNtKwJS6kwCgBWThcIQ6yI1jWEdrJYYFmHXJG/Rf/Nw8twEVh8l/Z0M";

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
        double overFlowAngle = 0;
        double adjustedAngle = currentHeading;  //Adjusted angle is to handle crossover of sign at 180 degrees
        double angleBufferForPrecision = 15;
        double throttleDownSpeed = 0.2;
        boolean throttledDown = false;
        //Create loop so robot spins until target angle is achieved
        //Based on the field coordinate system, positive roll is spinning to the left
        //But the drive vector equations consider turning to the right to be positive
        //So to establish a targetAngle, the desired spinAngle must be subtracted from the currentHeading
        double targetAngle = currentHeading - spinAngle;

        //The imu changes sign at 180 degrees
        // For this control loop to work, must catch this CROSSOVER event
        if (Math.abs(targetAngle)>180){
            currentHeadingModifier = true;
        }

        startSpinning(spinAngle, speed, motors);
        if(spinAngle > 0){
            //If spinning to the right, keep spinning while angle is greater than target angle
            // When spinning right, imu angle is decreasing
            while(opModeIsActive() && adjustedAngle > targetAngle) {
                //  Just keep spinning to the right
                if (currentHeadingModifier && currentHeading > 120) {
                    //This detects when crossover occurs, must add overflow
                    overFlowAngle = 180 - currentHeading;
                    adjustedAngle = -180 - overFlowAngle;
                } else {
                    adjustedAngle = currentHeading;
                }

                if (!throttledDown && Math.abs(adjustedAngle - targetAngle) < angleBufferForPrecision){
                    startSpinning(spinAngle,throttleDownSpeed, motors);
                }

                telemetry.addData("Target Spin", spinAngle);
                telemetry.addData("Target Angle", targetAngle);
                telemetry.addData("currentHeading", currentHeading);
                telemetry.addData("Status", "Turning");
                telemetry.update();
            }
        }else{
            //If spinning to the left, keep spinning while angle is less than target angle
            //When spinning left, the imu angle is increasing
            //so keep spinning while heading is less than target
            while (opModeIsActive() && adjustedAngle < targetAngle) {
                //  Just keep spinning to the left
                if(currentHeadingModifier && currentHeading < -120){
                    //If crossover occurs
                    overFlowAngle = 180 + currentHeading;  //add current heading since large negative
                    adjustedAngle = 180 + overFlowAngle;
                } else {
                    adjustedAngle = currentHeading;
                }

                if (!throttledDown && Math.abs(adjustedAngle - targetAngle) < angleBufferForPrecision){
                    startSpinning(spinAngle,throttleDownSpeed, motors);
                }

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

    private void turnToFieldHeading (double desiredFieldHeading, ArrayList<DcMotor> motors){
        // Field heading is the imu direction based on the assumed zero point from lander
        // This command calculates the turn inputs for twistToAngle function and calls it
        if((desiredFieldHeading>0 & currentHeading<0)
                | (desiredFieldHeading<0 && currentHeading>0)){
            // If the desired heading is different sign from currentHeading
            // Modify the desired field heading
            double overflowAngle = 180-Math.abs(desiredFieldHeading);
            if(currentHeading<0){
                desiredFieldHeading=-180-overflowAngle;
            } else{
                desiredFieldHeading = 180+overflowAngle;
            }
        }
        double requiredTurnAngle = desiredFieldHeading - currentHeading;  //How many angles must turn
        requiredTurnAngle *=-1;
        twistToAngle(requiredTurnAngle,0.6, motors);
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
        latchMotor.setDirection(DcMotor.Direction.REVERSE);
        latchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        latchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Initialize the variables that are being used in the main loop
        double spinAngle;    //NOTE:  Positive spin is towards right
        double driveX;
        double driveY;
        double spinSpeed;
        double twistToAngleSpeed = 0.35;
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
        //      b) Lands
        //      c) Moves away from lander a little
        //      d)Spins to face sampling area
        //  3)  Moves  to sample
        //
        //  4)  Lowers the latch to prepare for reattaching
        //  5)  Twists a little to align to depot
        //  6)  Lower arm angle to drop marker
        //      b) reverse collector motors to drop marker
        //  7) Retract arm angle to drive position
        //  8) If in the center, move back a little to avoid the other sampling objects
        //  9) Turn towards the crater
        //  10) Move to the crater

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
                        //Assume we only see the LEFT two
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

        armExtensionMotor.setTargetPosition(ARM_EXTENSION_COLLECTION_POSITION);
        armExtensionMotor.setPower(1);

        armAngleMotor.setTargetPosition(ARM_ANGLE_TRAVEL_POSITION);
        armAngleMotor.setPower(1);


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
        //  d) Twist the front of the robot to the sampling area
        twistToAngle(85, twistToAngleSpeed, motorList);

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  3) Move to sample
        //     Based on the position of the gold element, align to knock it off

        driveY = .85;         //Motion
        delayTime = SAMPLING_DRIVE_TIME;
        extraDelayTime = SAMPLING_EXTRA_DRIVE_TIME;

        //************************************
        if (goldPosition == "Left"){
            driveX = -SAMPLING_DRIVE_COMPONENT;         //Motion in y direction to get to left spot
            delayTime += extraDelayTime;        //Add extra drive time because longer distance to spot
        } else if (goldPosition == "Right"){
            driveX = SAMPLING_DRIVE_COMPONENT;
            delayTime += extraDelayTime;  //Add extra drive time because longer distance to spot
        } else {
            driveX = 0;
            driveY = 1;
        }
        spinSpeed = 0;      //This is used to determine how to spin the robot
        //This function will perform the drive step and stop the motors
        performDriveStep(driveX, driveY, spinSpeed, delayTime, motorList);


        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  4) Lowers the latch to prepare for reattaching
        //      After starting the drive step, move latch down to the DRIVE_POSITION
        latchMotor.setTargetPosition(LATCH_DRIVE_POSITION);
        latchMotor.setPower(1);


        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  5) Spin a little for better alignment if left or right
        spinAngle = SAMPLE_ALIGNMENT_TWIST_ANGLE;
        if (goldPosition == "Right"){
            spinAngle *= -1;
        }
        //Only spin if cube is left or right (not center)
        if (goldPosition == "Right" | goldPosition == "Left"){
            twistToAngle(spinAngle,twistToAngleSpeed,motorList);
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  6)  Lower arm angle to drop marker
        armAngleMotor.setTargetPosition(ARM_ANGLE_DUMP_POSITION);
        armAngleMotor.setPower(1);
        while(armAngleMotor.isBusy()){
            //Wait for the marker to be placed
            //We could add a touch sensor here to improve timing
            telemetry.update();
        }
        armAngleMotor.setPower(0);

        //  b) Spit out the marker
        collectMotor.setPower(-1);
        delayTime = 1000;
        performDriveStep(0, 0, 0, delayTime, motorList);  //delay for a second
        collectMotor.setPower(0);       //turn off the motor

        //  7) Retract arm angle to drive position
        armAngleMotor.setTargetPosition(ARM_ANGLE_TRAVEL_POSITION);
        armAngleMotor.setPower(1);
        telemetry.update();
        //Start raising the arm up to traveling position and then start moving.
        //Delay a small amount while moving
        delayTime = 1000;
        driveX = 0;
        driveY = 0;
        spinSpeed = 0;
        performDriveStep(driveX, driveY, spinSpeed, delayTime, motorList);  //Just a pause, no movement


        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  8) If in the center, move forward a little to avoid the other sampling objects
        if(goldPosition == "Center") {
            delayTime = (long) (SAMPLING_DRIVE_TIME*.7);
            driveX = 0;
            driveY = 1;
            spinSpeed = 0;
            performDriveStep(driveX, driveY, spinSpeed, delayTime, motorList);  //Just a pause, no movement

            delayTime = 400;
            driveX = 0;
            driveY = -1;
            spinSpeed = 0;
            performDriveStep(driveX, driveY, spinSpeed, delayTime, motorList);  //Just a pause, no movement

        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  9) Spin to face the crater
        //  The crater should be oriented 135 degrees to the right of the initialized position (-135 wrt gyro)
        //  To determine spin angle, subtract -135 (aka add 135)


        spinAngle = 120;
        if (goldPosition == "Left" | goldPosition == "Right") spinAngle += SAMPLE_ALIGNMENT_TWIST_ANGLE;
        if (goldPosition == "Right") spinAngle += 5;  //Add a little extra spin based on trials
        if(goldPosition == "Left") {
            spinAngle *= -1;
        }

        twistToAngle(spinAngle, twistToAngleSpeed, motorList);

        //turnToFieldHeading(45, motorList);
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  9) If coming from the center, move a little to avoid the lander leg
        if(goldPosition == "Center") {
            delayTime = CENTER_SAMPLE_CRATER_DRIVE_ALIGNMENT_TIME;
            driveX = -.7;  //Need to go mostly left
            driveY = .4;    //And forward some
            spinSpeed = 0;
            performDriveStep(driveX, driveY, spinSpeed, delayTime, motorList);  //Just a pause, no movement
        }


        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  10) Drive to the crater
        //  If center cube was sampled, add some drive time
        delayTime = CRATER_DRIVE_TIME;
        if (goldPosition=="Left") delayTime += EXTRA_CRATER_DRIVE_TIME;
        driveX = 0;
        driveY = 1;
        spinSpeed = 0;
        performDriveStep(driveX, driveY, spinSpeed, delayTime, motorList);  //Just a pause, no movement

        //Make sure arm is fully extended and armAngle is in desired location before ending
        while (armAngleMotor.isBusy() | armExtensionMotor.isBusy()){
            //Wait for it to complete
        }
    }
}
