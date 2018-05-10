package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.util.Range.clip;


@TeleOp(name="TOPTankMode", group="Iterative Opmode")
//@Disabled
public class TOPTankMode extends OpMode
{

    //astea-s pt brat
    //static final double INCREMENT   = 0.01;     // cu cat se muta servoul per cycle(in whileul ala )
    //static final int    CYCLE_MS    =   50;     // perioada unui cycle (yay fizica)
    //static final double MAX_POS     =  0.8;     // pozitia max
   // static final double MIN_POS     =  0.0;     // pozitia min
    HardwareMap         robot   = new HardwareMap();   // hardware mapul de la robot
    ElapsedTime     runtime = new ElapsedTime();
    //initializarea robotului
    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initializat");

        //aicea se conecteaza obiectul declarat cu ce nume i-am dat la setari la robot in
        //robot controller app. denumirea tre sa fie aceeasi


        //fiind puse in oglinda, unul din motoare merge pe dos si tre inversat... altfel face cercuri

        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // anunta
        telemetry.addData("Status", "Robot initializat!");
        telemetry.addData("Controale:", "lift bumpere, bratTotem triggere");
        telemetry.addData("Controale2:", "X brat A cub Y totem");
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * asta se ruleaza pana se apasa stop
     */

    @Override
    public void loop() {
        // tinut evidenta viteza

        double leftPower;
        double rightPower;
        double liftPower=0;
        double totemArmPower=0;


        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        //double drive = -gamepad1.left_stick_y;
        //double turn  =  gamepad1.right_stick_x;
        //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;


        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
         leftPower  = gamepad1.left_stick_y ;
         rightPower = gamepad1.right_stick_y ;

         //pt macara
         //bumper stanga coboara macaraua, bumper dreapta urca macaraua
        if(gamepad1.right_bumper == true)
            liftPower+=robot.liftSpeed;
        else if(gamepad1.left_bumper == true)
            liftPower-=robot.liftSpeed;
        else liftPower=0;
        //clip sa nu dea inafara la 0,1
        if (gamepad1.left_trigger > 0) {
            totemArmPower=robot.totemArmSpeed;
        }
        if (gamepad1.right_trigger > 0) {
            totemArmPower=-robot.totemArmSpeed;
        }
        totemArmPower=clip(liftPower,-1.0,1.0);


        // Send calculated power to wheels
        robot.setMotorPowers(leftPower,rightPower);
        robot.liftMotor.setPower(liftPower);
        robot.totemArmMotor.setPower(totemArmPower);

        //servo stuff


        if(gamepad1.x) {
            if(robot.armStatus=="lowered")
                robot.liftArm();
            else
                robot.lowerArm();
            telemetry.addData("arm", robot.armStatus);
            robot.liftArm();
        }
        if (gamepad1.y) {
            robot.lowerArm();
        }
        if (gamepad1.a) {
            if(robot.clawStatus=="closed")
                 robot.openClaw();
            else
                robot.closeClaw();
            telemetry.addData("cubeClaw",robot.clawStatus);

        }




        telemetry.update();
        // Show the elapsed game time and wheel power.
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
