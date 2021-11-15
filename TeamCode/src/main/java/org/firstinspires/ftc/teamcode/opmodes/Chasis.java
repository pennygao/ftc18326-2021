package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;


@TeleOp(name="MainChassis", group="Linear Opmode")
public class Chasis extends LinearOpMode {
    //private BNO055IMU imu = null;
    //BNO055IMU.Parameters parameters = null;


    //public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor topleft = null;
    private DcMotor topright = null;
    private DcMotor botleft = null;
    private DcMotor botright = null;
    private DcMotor arm=null;
    private DcMotor encoder = null;

    long lastTimeStamp;
    int sensitivity = 1000;
    int state=0;

    @Override
    public void runOpMode() {
        //telemetry.addData("Status", "Initialized");
        //telemetry.update();
        lastTimeStamp = System.currentTimeMillis();
/*
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
*/

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        botleft  = hardwareMap.get(DcMotor.class, "bl");
        botright = hardwareMap.get(DcMotor.class, "br");
        topleft  = hardwareMap.get(DcMotor.class, "tl");
        topright = hardwareMap.get(DcMotor.class, "tr");
        arm = hardwareMap.get(DcMotor.class, "gun");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        topleft.setDirection(DcMotor.Direction.FORWARD);
        botleft.setDirection(DcMotor.Direction.FORWARD);
        topright.setDirection(DcMotor.Direction.REVERSE);
        botright.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        int toggle = 2;
        boolean pbutton=false;
        String message="";
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
                /*
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x/2;
            leftPower    = Range.clip(0.5*drive + 0.5*turn, -0.5, 0.5) ;
            rightPower   = Range.clip(0.5*drive - 0.5*turn, -0.5, 0.5) ;
            */

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
                /*
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);*/

            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            //telemetry.addData("Power", "Left 1stick y (%.2f), Right stick x (%.2f)", -drive, turn*2);
            telemetry.update();
            //*/
            //forward+backwards
            topleft.setPower(-1*gamepad1.left_stick_y);
            topright.setPower(-1*gamepad1.left_stick_y);
            botleft.setPower(-1*gamepad1.left_stick_y);
            botright.setPower(-1*gamepad1.left_stick_y);

            //left and right
            topleft.setPower(1*gamepad1.left_stick_x);
            topright.setPower(-1*gamepad1.left_stick_x);
            botleft.setPower(-1*gamepad1.left_stick_x);
            botright.setPower(1*gamepad1.left_stick_x);

            //rotate
            topleft.setPower(1*gamepad1.right_stick_x);
            topright.setPower(-1*gamepad1.right_stick_x);
            botleft.setPower(1*gamepad1.right_stick_x);
            botright.setPower(-1*gamepad1.right_stick_x);

            //"Arm"

            if (gamepad1.left_bumper  /*&& System.currentTimeMillis() - lastTimeStamp > sensitivity*/) {
                if (state==0 ){
                    arm.setPower(1);
                    if (arm.getCurrentPosition()>=10000){
                        arm.setPower(0);
                    }
                }
                else if (state==1 ){
                    arm.setPower(-1);
                    if (arm.getCurrentPosition()<0){
                        arm.setPower(0);
                    }
                }
                state=1-state;
            }
/*
            if (gamepad1.right_bumper  && System.currentTimeMillis() - lastTimeStamp > sensitivity)
                arm.setPower(-1);
*/
            telemetry.addData("rightStick on", "Pressed?: " + gamepad1.right_stick_x);
            //telemetry.addData("status", "pressed: " + message);
            //telemetry.addData("Previous", "Prev. B status: " );
            //Orientation angle = imu.getAngularOrientation();

//          telemetry.addData("Angle Orientation: ", "%.2f, %.2f, %.2f", angle.firstAngle, angle.secondAngle, angle.thirdAngle);

            telemetry.update();

        }
    }
}

