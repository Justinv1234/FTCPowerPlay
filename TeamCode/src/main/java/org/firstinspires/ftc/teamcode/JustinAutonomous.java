package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
 @Autonomous(name = "Justin's Autonomous")
public class JustinAutonomous extends LinearOpMode
{
    private OpenCvCamera                camera;
    private DcMotor                     Motor1 = null;
    private DcMotor                     Motor2 = null;

    private ElapsedTime                 runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1301 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                                   (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    AprilTagDetectionPipeline aprilTagDetectionPipeline; //Pipeline

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        //Maps both motors that controls the robot's movement
        Motor1 = hardwareMap.get(DcMotor.class, "Motor1"); 
        Motor2 = hardwareMap.get(DcMotor.class, "Motor2");

        Motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor2.setDirection(DcMotorSimple.Direction.FORWARD);

        Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Sets up and configures the camera to detect AprilTags
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy); //

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                
            }
        });

        telemetry.setMsTransmissionInterval(50);

        //Detects all AprilTags that the camera can see
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections(); //Stores in ArrayList

            if(currentDetections.size() != 0) //If there aren't any found
            {
                boolean tagFound = false; //set false

                for(AprilTagDetection tag : currentDetections) //   for every detection
                {
                    if(tag.id == 1 || tag.id == 2 || tag.id == 3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                //Communicates to driver if a tag is or was present and when it was last seen.
                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Robot DOESN'T see the cone please move the robot until this text goes away!!");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The cone has NEVER been seen)");
                }
                else
                {
                    telemetry.addLine("\nWe HAVE seen the cone before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Cone snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No cone snapshot available");
            telemetry.update();
        }

        /* Following code moves based of latest snapshot and drives the robot to the according parking space*/
        if(tagOfInterest != null)
        {
            if (tagOfInterest.id == 1)
            {
                encoderDrive(DRIVE_SPEED,  3.5,  3.5, 5.0); //1
                encoderDrive(TURN_SPEED,   -8.5, 8.5, 4.0);//2
                encoderDrive(DRIVE_SPEED,  15.5,  15.5, 5.0);//3         //Drive to the first parking space 
                encoderDrive(TURN_SPEED,   7, -7, 4.0);//4
                encoderDrive(DRIVE_SPEED,  20,  20, 5.0);//5
            }
            else if (tagOfInterest.id == 2)
            {
                encoderDrive(DRIVE_SPEED, 25, 25, 4);                   //Drive to the third parking space
            }
            else if (tagOfInterest.id == 3)
            {
                encoderDrive(DRIVE_SPEED,  3.5,  3.5, 5.0); //1
                encoderDrive(TURN_SPEED,   8, -8, 4.0);//2
                encoderDrive(DRIVE_SPEED,  16.25,  16.25, 5.0);//3     //Drive to the third parking space
                encoderDrive(TURN_SPEED,   -8, 8, 4.0);//4
                encoderDrive(DRIVE_SPEED,  20,  20, 5.0);//5
            }
        }
    }

    //Display information for the driver
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = Motor1.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = Motor2.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            Motor1.setTargetPosition(newLeftTarget);
            Motor2.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            Motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            Motor1.setPower(Math.abs(speed));
            Motor2.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (Motor1.isBusy() && Motor2.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        Motor1.getCurrentPosition(), Motor2.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            Motor1.setPower(0);
            Motor2.setPower(0);

            // Turn off RUN_TO_POSITION
            Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

}