package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;

public class EruditeUtils {
    public void claw(Servo leftClaw, Servo rightClaw, double left_claw_position,
                     double right_claw_position, double diffClaw){
        leftClaw.setPosition(left_claw_position - diffClaw);
        rightClaw.setPosition(right_claw_position + diffClaw);
    }
}

