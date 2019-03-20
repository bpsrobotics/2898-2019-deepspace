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
    val shooter = DoubleSolenoid(5, 6)
    val cargoTalon = TalonWrapper(CARGOINTAKE)

    var isCargoIn = true


    var hatch = false

    init {
        AsyncLooper(50.0) {
            SmartDashboard.putNumber("cargo A", cargoTalon.outputCurrent)
        }.start()

        AsyncLooper(50.0) {
            if (OI.opCtl.getRawButton(3) || OI.opCtl.getRawButton(1)) {
                shooter.set(DoubleSolenoid.Value.kReverse)
            } else {
                shooter.set(DoubleSolenoid.Value.kForward)
            }
            if (OI.opCtl.getRawButton(2) || OI.opCtl.getRawButton(1)) {
                hatchSelenoid.set(DoubleSolenoid.Value.kReverse)
            } else {
                hatchSelenoid.set(DoubleSolenoid.Value.kForward)
            }
            cargoTalon.set(ControlMode.PercentOutput, OI.deadzone(-1.0 * OI.opCtl.getRawAxis(1)))
        }.start()
    }

}