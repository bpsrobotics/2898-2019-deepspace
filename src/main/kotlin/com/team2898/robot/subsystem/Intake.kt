package com.team2898.robot.subsystem

import com.ctre.phoenix.motorcontrol.ControlMode
import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.Subsystem
import com.team2898.engine.motion.TalonWrapper
import com.team2898.robot.OI
import com.team2898.robot.config.CARGOINTAKE
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlin.math.abs


object Intake: Subsystem(50.0, "intake") {
    override val enableTimes: List<GamePeriods> = listOf(GamePeriods.TELEOP, GamePeriods.AUTO)

    val hatchSelenoid = DoubleSolenoid(0, 1)
    val cargoTalon = TalonWrapper(CARGOINTAKE)

    var isCargoIn = true


    var hatch = false

    init {
        AsyncLooper(50.0) {
            SmartDashboard.putNumber("cargo A", cargoTalon.outputCurrent)
        }.start()

        AsyncLooper(50.0) {
//            if (!isCargoIn) {
//                cargoTalon.set(ControlMode.PercentOutput, -0.3)
//                isCargoIn = cargoTalon.outputCurrent > 4.0
//            } else cargoTalon.set(ControlMode.PercentOutput, OI.opCtl.getRawAxis(1))
            cargoTalon.set(ControlMode.PercentOutput, -1 * OI.opCtl.getRawAxis(1))
        }.start()
    }

    fun hatchIntake() {
        if (hatch) hatchSelenoid.set(DoubleSolenoid.Value.kReverse)
        else hatchSelenoid.set(DoubleSolenoid.Value.kForward)
    }


    fun cargoReset() {
        isCargoIn = false
    }
}