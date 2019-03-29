package com.team2898.robot.commands

import com.ctre.phoenix.motorcontrol.ControlMode
import com.team2898.robot.subsystem.Arm
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.command.Command

object Safety: Command() {
    val limit = DigitalInput(0)
    override fun isFinished(): Boolean = limit.get()

    override fun initialize() {
        Arm.test = false
        Arm.controlLoop.end()
    }

    override fun execute() {
        Arm.leftMaster.set(ControlMode.PercentOutput, 0.2)
        println("safety")
    }

    override fun end() {
        Arm.test = true
        Arm.leftMaster.set(ControlMode.PercentOutput, 0.0)
        Arm.armEnc.reset()
        Arm.controlLoop.start()
    }
}