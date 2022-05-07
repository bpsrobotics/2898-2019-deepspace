//package com.team2898.robot.commands.testbot
//
//import com.team2898.engine.motion.CheesyDrive
//import com.team2898.engine.motion.DriveSignal
//import com.team2898.robot.OI
//import com.team2898.robot.commands.Teleop
//import com.team2898.robot.config.ArmConf.*
//import com.team2898.robot.subsystem.Arm
//import com.team2898.robot.subsystem.DiscBrake
//import com.team2898.robot.subsystem.Drivetrain
//import edu.wpi.first.wpilibj.command.Command
//import kotlinx.serialization.ImplicitReflectionSerializer
//
//class PIDDrive: Command() {
//    val Kp = 0.5
//    val Ki = 0.3
//    val kd = 0.2
//
//    var prevError = Double.NaN
//    var Cp = Double.NaN
//    var Ci = Double.NaN
//    var Cd = Double.NaN
//
//    override fun execute() {
//        DiscBrake.brakeUpdate()
//        if (OI.opCtl.getRawButton(7)) Arm.updateTarget(hatchl3 + 0.25)
//        if (OI.opCtl.getRawButton(9)) Arm.updateTarget(hatchl2 + 0.25)
//        if (OI.opCtl.getRawButton(11)) Arm.updateTarget(hatchl1 + 0.17)
//        if (OI.opCtl.getRawButton(8)) Arm.updateTarget(cargol3 - 0.1)
//        if (OI.opCtl.getRawButton(10)) Arm.updateTarget(cargol2 - 0.1)
//        if (OI.opCtl.getRawButton(12)) Arm.updateTarget(cargol1 - 0.1)
//        if (OI.opCtl.getRawButton(5)) Arm.updateTarget(cargo)
//        if (OI.driverController.aButton) Arm.updateTarget(0.0)
//        if (OI.driverController.bButton || OI.opCtl.pov == 0) Arm.updateTarget(Arm.currentPos.pos() + 0.1)
//        if (OI.driverController.xButton || OI.opCtl.pov == 180) Arm.updateTarget(Arm.currentPos.pos() - 0.1)
//
//        var power = CheesyDrive.updateCheesy(
//                (if (!OI.quickTurn) OI.turn else -OI.leftTrigger + OI.rightTrigger),
//                -OI.throttle,
//                OI.quickTurn,
//                false
//        )
//
//        Drivetrain.openLoopPower(power)
//    }
//
//    override fun isFinished(): Boolean {
//        return !OI.driverController.getRawButton(5)
//    }
//
//    override fun initialize() {
//        reset()
//    }
//
//    fun reset() {
//        prevError = Double.NaN
//        Cp = Double.NaN
//        Ci = Double.NaN
//        Cd = Double.NaN
//    }
//
//
//}