/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

//import com.qualcomm.robotcore.hardware.HardwareMap;


@Autonomous(name="Autonom v4 BLUE", group="Autonom")

public class AutonomV4Blue extends AutonomV4 {

    // Declare OpMode members.
    private HardwareMap robot = new HardwareMap();
    public ElapsedTime runtime = new ElapsedTime();
    Orientation             lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    String allianceColor="red", ballColor;
    VuMarkFinder        vmf;
    RelicRecoveryVuMark vuMark;
    DriveByEncoder drive = new DriveByEncoder();
    int keyRow=-2;
    double turn_speed=0.5, drive_speed=0.6;

    double value90DegreeTurn=4;
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //init chestie vuforia
        vmf = new VuMarkFinder(hardwareMap, "RelicVuMark", true, VuforiaLocalizer.CameraDirection.BACK);

        //gyroscop


        while (!isStopRequested())
        {
            sleep(50);
            idle();
        }

        //aici init motoare
        //aici setat directii

        // Wait for the game to start (driver presses PLAY)

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        robot.leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        runtime.reset();



        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.closeClaw();
        //citit cod vuforia
        vmf.activate();

        if (vmf.findVuMark())
        {
            // Convert vumark instance  id to game specific id.
            vuMark = RelicRecoveryVuMark.from(vmf.instanceId);

            //debug stuff
            telemetry.addData("VuMark", "%s visible", vuMark);
            telemetry.addData("Pose", vmf.formatPose(vmf.pose));
         //   telemetry.addData("X Y Z", "X=%f  Y=%f  Z=%f", vmf.tX, vmf.tY, vmf.tZ);

            if(vuMark==RelicRecoveryVuMark.LEFT)
                keyRow=1;
            else if(vuMark==RelicRecoveryVuMark.CENTER)
                keyRow=2;
            else keyRow=3;

        }
        else
        { telemetry.addData("VuMark", "not visible"); keyRow=2;}


        //extins mana

                robot.lowerArm();
             sleep(2000);
        //TODO:rotit fata spate in functie de ce bila e in fata senzorului de culoare de pe mana

            do
            {
                ballColor = detectColor(robot.armColor);
            } while (ballColor=="no");

            int sign;
            if(ballColor=="red") sign=-1; else sign=1;
            telemetry.addData("Culoare Bila in spate:", ballColor);
            telemetry.update();
            drive.encoderDrive(turn_speed,2*sign,2*sign*-1,2,robot,this);
            sleep(6000);
            drive.encoderDrive(turn_speed,2*sign*-1,2*sign,2,robot,this);


        //ridicat mana inapoi
            robot.liftArm();
            sleep(2000);

            //mers in fata 25.5 inch

            drive.encoderDrive(1,-25.5,25.5,7,robot,this);

        //daca a dat de banda rosie, rotit 90 gr
            if(detectColor(robot.groundColor) == "red"){
                telemetry.addData("Culoare Banda", allianceColor);
                telemetry.update();
                drive.encoderDrive(turn_speed,value90DegreeTurn*-1,value90DegreeTurn,2,robot,this);


                drive.encoderDrive(1,-2,2,6,robot,this);

                robot.openClaw();
                drive.encoderDrive(0.3,2,-2,6,robot,this);
            }
            else
            {
                drive.encoderDrive(turn_speed,value90DegreeTurn,value90DegreeTurn*-1,2,robot,this);


                drive.encoderDrive(1,-10,10,6,robot,this);


                drive.encoderDrive(turn_speed,value90DegreeTurn*-1,value90DegreeTurn,2,robot,this);


                drive.encoderDrive(1,-2,2,6,robot,this);


                robot.openClaw();
                sleep(3000);

                drive.encoderDrive(0.3,2,-2,6,robot,this);


            }











/*
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
        */
    }

    public String detectColor(ColorSensor s)
    {
        if(s.blue() + s.red() > 20)
        {
            if(s.blue() > s.red()) return "blue";
            else return "red";
        }
        return "none";
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

}





