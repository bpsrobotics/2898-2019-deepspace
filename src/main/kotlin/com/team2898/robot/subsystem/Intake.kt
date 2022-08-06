package com.team2898.robot.subsystem

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.Subsystem
import com.team2898.engine.motion.TalonWrapper
import com.team2898.robot.OI
import com.team2898.robot.config.*
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlin.math.sign


object Intake: Subsystem(50.0, "intake") {
    override val enableTimes: List<GamePeriods> = listOf(GamePeriods.TELEOP, GamePeriods.AUTO)

    // grabber
    private val hatchSelenoid = DoubleSolenoid(HATCH)
    // intake brake
    private val latching = DoubleSolenoid(LATCH)
    // hatch launchers
    private val shooter = DoubleSolenoid(SHOOT)

    private val cargoTalon = TalonWrapper(CARGO_INTAKE)

    /** Number of 'ticks' that the cargo talon has been over the current limit for. */
    var overCurrentTicks = -1

    var autoIntake = false
    var isHatchAuto = false

    private val hatchLimit = DigitalInput(7)
    init {
        cargoTalon.setNeutralMode(NeutralMode.Brake)

        AsyncLooper(50.0) {
            SmartDashboard.putNumber("cargo A", cargoTalon.statorCurrent)
        }.start()

        AsyncLooper(50.0) {
            if (overCurrentTicks > -1) overCurrentTicks++
            val isHatchIn = !hatchLimit.get()

            val shooterState = OI.opCtl.getRawButton(1)
            var hatchState = (OI.opCtl.getRawButton(2) || OI.opCtl.getRawButton(1)) || isHatchAuto
            var latchState = (OI.opCtl.getRawAxis(1) < 0.5 && OI.opCtl.getRawAxis(1) > -0.5)
            var cargoValue = if (latchState) 0.0 else sign(OI.opCtl.getRawAxis(1)) * -0.8

            if (overCurrentTicks > 6) {
                reset()
                cargoValue = 0.0
            }

            if (isHatchIn && isHatchAuto) {
                isHatchAuto = false
            }
            if (isHatchAuto) hatchState = true


            if (OI.opCtl.getRawButton(4) || autoIntake) {
                autoIntake = true
                latchState = false
                cargoValue = -0.7
            }

            if ((cargoTalon.statorCurrent > 20.0 && autoIntake) || OI.opCtl.getRawButton(6)) {
                if (overCurrentTicks < 0) overCurrentTicks = 0
                autoIntake = false
                latchState = true
            }

            updateShooter(shooterState)
            updateHatch(hatchState)
            updateLatch(latchState)
            updateCargoTalon(cargoValue)
        }.start()
    }

    private fun updateShooter(state: Boolean) {
        if (state) shooter.set(DoubleSolenoid.Value.kReverse)
        else shooter.set(DoubleSolenoid.Value.kForward)
    }

    private fun updateHatch(state: Boolean) {
        if (state) hatchSelenoid.set(DoubleSolenoid.Value.kReverse)
        else hatchSelenoid.set(DoubleSolenoid.Value.kForward)
    }

    private fun updateLatch(state: Boolean) {
        if (state) latching.set(DoubleSolenoid.Value.kReverse)
        else latching.set(DoubleSolenoid.Value.kForward)
    }

    private fun updateCargoTalon(value: Double) {
        cargoTalon.set(ControlMode.PercentOutput, value)
    }

    fun reset() {
        autoIntake = false
        overCurrentTicks = -1
    }
}

//            if ((OI.opCtl.getRawAxis(1) < 0.5 && OI.opCtl.getRawAxis(1) > -0.5)) {
//                latching.set(DoubleSolenoid.Value.kReverse)
//            } else {
//                latching.set(DoubleSolenoid.Value.kForward)
//            }

//            if (OI.opCtl.getRawAxis(1) < 0.6 && OI.opCtl.getRawAxis(1) > -0.5) {
//                cargoTalon.set(ControlMode.PercentOutput, 0.0)
//            } else {
//                cargoTalon.set(ControlMode.PercentOutput, -0.5 * OI.opCtl.getRawAxis(1))
//            }
//            if (OI.opCtl.getRawButton(3) || OI.opCtl.getRawButton(1)) {
//                shooter.set(DoubleSolenoid.Value.kReverse)
//            } else {
//                shooter.set(DoubleSolenoid.Value.kForward)
//            }
//            if (OI.opCtl.getRawButton(2) || OI.opCtl.getRawButton(1) || hatch) {
//                hatchSelenoid.set(DoubleSolenoid.Value.kReverse)
//            } else {
//                hatchSelenoid.set(DoubleSolenoid.Value.kForward)
//            }
