package com.team2898.robot.subsystem

import com.ctre.phoenix.motorcontrol.ControlMode
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.Subsystem
import com.team2898.engine.motion.TalonWrapper
import com.team2898.robot.config.CARGOINTAKE
import edu.wpi.first.wpilibj.DoubleSolenoid


object Intake: Subsystem(50.0, "intake") {
    override val enableTimes: List<GamePeriods> = listOf(GamePeriods.TELEOP, GamePeriods.AUTO)

    val hatchSelenoid = DoubleSolenoid(0, 1)
    val cargoTalon = TalonWrapper(CARGOINTAKE)


    var hatch = false

    fun hatchIntake() {
        if (hatch) hatchSelenoid.set(DoubleSolenoid.Value.kReverse)
        else hatchSelenoid.set(DoubleSolenoid.Value.kForward)
    }

    fun cargoIntake(value: Double) {
        cargoTalon.set(ControlMode.PercentOutput, value)
    }
}