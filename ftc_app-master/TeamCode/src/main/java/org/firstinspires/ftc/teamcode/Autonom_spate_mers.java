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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

//import com.qualcomm.robotcore.hardware.HardwareMap;


@Autonomous(name="Autonom SPATEmerssiatat", group="Autonom")

public class Autonom_spate_mers extends AutonomV4 {

    // Declare OpMode members.
    private HardwareMap robot = new HardwareMap();
    public ElapsedTime runtime = new ElapsedTime();
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    String allianceColor = "red", ballColor;
    VuMarkFinder vmf;
    RelicRecoveryVuMark vuMark;
    DriveByEncoder drive = new DriveByEncoder();
    int keyRow = -2;
    double turn_speed = 0.5, drive_speed = 0.6;

    double value90DegreeTurn = 4;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        // robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //  robot.rightDrive.setMode(DcMotor.RunMode.);

        //init chestie vuforia
        //vmf = new VuMarkFinder(hardwareMap, "RelicVuMark", true, VuforiaLocalizer.CameraDirection.BACK);

        //gyroscop


        //aici init motoare
        //aici setat directii

        // Wait for the game to start (driver presses PLAY)

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        waitForStart();
        runtime.reset();


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.closeClaw();

        //extins mana

        robot.lowerArm();
        sleep(2000);
        //TODO:rotit fata spate in functie de ce bila e in fata senzorului de culoare de pe mana
        // drive.encoderDrive(0.7,5.5,5.5,7,robot,this);
        robot.driveForward(0.5,-1.0,this);
        robot.liftArm();
        robot.driveForward(4.5,-1.0,this);
        sleep(30000);
      //  driveForward(10);
        //drive.encoderDrive(0.7, 20, 20, 7, robot, this);













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

    public String detectColor(ColorSensor s) {
        if (s.blue() + s.red() > 20) {
            if (s.blue() > s.red()) return "blue";
            else return "red";
        }
        return "none";
    }

}