package com.team2898.robot.commands.testbot

import com.team2898.engine.math.clamp
import com.team2898.engine.motion.CheesyDrive
import com.team2898.engine.motion.DriveSignal
import com.team2898.engine.subsystems.Navx
import com.team2898.robot.OI
import com.team2898.robot.subsystem.Drivetrain
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.ghrobotics.lib.mathematics.units.degree

class TestTeleop: Command() {
    override fun isFinished(): Boolean  =false

    override fun initialize() {
        Drivetrain.encoders {
            reset()
        }
    }

    override fun execute() {
        var power = CheesyDrive.updateCheesy(
                (if (!OI.quickTurn) OI.turn else -OI.leftTrigger + OI.rightTrigger),
                -OI.throttle,
                OI.quickTurn,
                false
        )
        if (OI.driverController.pov == 90) {
            power = DriveSignal(left = 0.3, right = -0.1)
        }
        if (OI.driverController.pov == 270) {
            power = DriveSignal(left = -0.1, right = 0.3)
        }
        if (OI.driverController.pov == 0 || OI.driverController.pov == 360) {
            power = DriveSignal(0.2, 0.2)
        }
        if (OI.driverController.pov == 180) {
            power = DriveSignal(-0.2, -0.2)
        }
        if (OI.driverController.getRawButton(5)) {
            val error = (SmartDashboard.getNumber("point_to_target", 0.0) + SmartDashboard.getNumber("robot_to_target_angle", 0.0)).degree
            var k = (error.degree/180 * 0.8)/2
            power = DriveSignal(clamp(power.left + k, -1.0, 1.0), clamp(power.right - k, -1.0, 1.0))
        }

        if (OI.driverController.getRawButton(6)) {
            power = DriveSignal(brake = true)
        }
        SmartDashboard.updateValues()
        Drivetrain.openLoopPower(power)
    }

/*    override fun execute() {
        Drivetrain.openLoopPower(DriveSignal(OI.driverController.getRawAxis(1), OI.driverController.getRawAxis(5)))
    }*/
}