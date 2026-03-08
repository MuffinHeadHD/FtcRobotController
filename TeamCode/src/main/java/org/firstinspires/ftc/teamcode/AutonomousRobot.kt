package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.parts.IntakeMode

class AutonomousRobot(opMode: OpMode): Robot(opMode) {
    inner class SpindexerRotate(val rotation: Int): Action {
        private var ran = false
        private val timer = ElapsedTime()

        override fun run(p: TelemetryPacket): Boolean {
            if (!ran) {
                p.put("Spindexer (first)", spindexer.isFinished())
                spindexer.rotate(rotation)
                timer.reset()
                ran = true
                return true
            } else {
                p.put("Spindexer (ran)", spindexer.isFinished())
                p.put("Spindexer (servo)", spindexer.servo.servo.power)
                p.put("Spindexer (target)", spindexer.servo.targetPosition)
                p.put("Spindexer (current)", spindexer.servo.position)
                p.put("Spindexer (time)", timer.seconds())
                return !spindexer.isFinished() && timer.seconds() > 0.35
            }
        }
    }

    inner class SetIntakeIn(): Action {
        override fun run(p: TelemetryPacket): Boolean {
            intake.set(IntakeMode.IN)
            return false
        }
    }

    inner class SetIntakeOut(): Action {
        override fun run(p: TelemetryPacket): Boolean {
            intake.set(IntakeMode.OUT)
            return false
        }
    }

    inner class SetIntakeOff(): Action {
        override fun run(p: TelemetryPacket): Boolean {
            intake.set(IntakeMode.OFF)
            return false
        }
    }

    inner class Update(): Action {
        override fun run(p: TelemetryPacket): Boolean {
            update()
            return true
        }
    }
}