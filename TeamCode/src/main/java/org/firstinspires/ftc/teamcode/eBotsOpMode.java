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
        double angleBufferForPrecision = 15;
        double throttleDownSpeed = 0.2;
        boolean throttledDown = false;
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

        startSpinning(spinAngle, speed, motors);
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
                if (!throttledDown && Math.abs(adjustedAngle - targetAngle) < angleBufferForPrecision){
                    startSpinning(spinAngle,throttleDownSpeed, motors);
                    throttledDown = true;
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

                if (!throttledDown && Math.abs(adjustedAngle - targetAngle) < angleBufferForPrecision){
                    startSpinning(spinAngle,throttleDownSpeed, motors);
                    throttledDown = true;
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



                    //Classify the position of each mineral




                    //Look at gyro to see if the robot has spun some
                    //And use that to guess whether seeing left 2 or right 2 objects
                    //Note:  spinning to the left is positive heading, right is negative
                    //This also assumes objects are centered initially

                    //double headingWhileLowering = getCurrentHeading();
                    /*GoldPosition assumedLeftPosition = GoldPosition.UNKNOWN;
                    GoldPosition assumedRightPosition = GoldPosition.UNKNOWN;
                    GoldPosition assumedMissingPosition = GoldPosition.UNKNOWN;

                    if(headingWhileLowering >= 0){
                        //If robot has swung left slightly
                        assumedLeftPosition = GoldPosition.LEFT;
                        assumedRightPosition = GoldPosition.CENTER;
                        assumedMissingPosition = GoldPosition.RIGHT;

                    }  else{
                        //If robot has swung right slightly
                        assumedLeftPosition = GoldPosition.CENTER;
                        assumedRightPosition = GoldPosition.RIGHT;
                        assumedMissingPosition = GoldPosition.LEFT;
                    }*/

                    //Infer where gold is if only 2 objects recognized
                    //Assume we only see the LEFT two
                    /*if (goldMineralX == -1 && silverMineral1X != -1){
                        //If gold isn't one of the two objects
                        goldPosition = assumedMissingPosition;
                    }  else if (goldMineralX != -1 && silverMineral1X != -1){
                        //If a gold and silver are visible
                        if (goldMineralX < silverMineral1X){
                            //If gold is further left, assume left
                            goldPosition = assumedLeftPosition;
                        } else {
                            //If gold is further right, assume center
                            goldPosition = assumedRightPosition;
                        }
                    }*/

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
