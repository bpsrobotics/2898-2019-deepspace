package com.team2898.engine.motion

import edu.wpi.first.wpilibj.Timer
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sign
import kotlin.math.sqrt


data class Constrains(val maxVel: Double, val maxAcc: Double)
data class ProfilePos(val position: Double, val velocity: Double)
enum class Shape { Trap, Tri }

class TrapezoidProfile(
        val constrains: Constrains,
        val initial: ProfilePos) {
    var oldTime = Timer.getFPGATimestamp()
    var lastTime = oldTime
    var dt = 0.0
    var signM = 1.0
    var shape = Shape.Tri
    var isFinished = false
    var oldPosRef = 0.0
    var oldPos = 0.0
    var oldVel = 0.0
    var pos = 0.0
    var vel = 0.0
    var acc = 0.0
    var tBrk = 0.0
    var dBrk = 0.0
    var tAcc = 0.0
    var dAcc = 0.0
    var dTot = 0.0
    var tDec = 0.0
    var dDec = 0.0
    var dVel = 0.0
    var tVel = 0.0
    var velSt = 0.0

    fun update(posRef: Double): Double {
        if (oldPosRef != posRef) {
            isFinished = false
            oldPosRef = posRef
            oldPos = pos
            oldVel = vel;
            oldTime = lastTime;

            tBrk = abs(oldVel) / constrains.maxAcc
            dBrk = tBrk * abs(oldVel) / 2

            signM = sign(posRef - (oldPos + sign(oldVel) * dBrk))

            if (signM != sign(oldVel)) {
                tAcc = (constrains.maxVel / constrains.maxAcc)
                dAcc = tAcc * (constrains.maxVel / 2)
            } else {
                tBrk = 0.0
                dBrk = 0.0
                tAcc = (constrains.maxVel - abs(oldVel)) / constrains.maxAcc
                dAcc = tAcc * (constrains.maxVel + abs(oldVel)) / 2
            }

            dTot = abs(posRef - oldPos + signM * dBrk)

            tDec = constrains.maxVel / constrains.maxAcc
            dDec = tDec * constrains.maxVel / 2
            dVel = dTot - (dAcc + dDec)
            tVel = dVel / constrains.maxVel


            if (tVel > 0) shape = Shape.Trap
            else {
                shape = Shape.Tri

                if (signM != sign(oldVel)) {
                    velSt = sqrt(constrains.maxAcc * dTot)
                    tAcc = (velSt / constrains.maxAcc);
                    dAcc = tAcc * (velSt / 2)
                } else {
                    tBrk = 0.0
                    dBrk = 0.0
                    dTot = abs(posRef - oldPos)
                    velSt = sqrt(0.5 * oldVel * oldVel + constrains.maxAcc * dTot)
                    tAcc = (velSt - abs(oldVel)) / constrains.maxAcc
                    tAcc = tAcc * (velSt + abs(oldVel)) / 2
                }
                tDec = velSt / constrains.maxAcc
                dDec = dDec * velSt / 2
            }
        }

        val currentTIme = Timer.getFPGATimestamp()
        dt = currentTIme - oldTime
        calcProfile(posRef)
        lastTime = currentTIme
        return pos
    }

    fun calcProfile(posRef: Double) {
        when (shape) {
            Shape.Trap -> {
                if (dt <= (tBrk + tAcc)) {
                    pos = oldPos + oldVel * dt + signM * 0.5 * constrains.maxAcc * dt * dt
                    vel = oldVel + signM * constrains.maxAcc * dt
                    acc = signM * constrains.maxAcc
                } else if (dt > (tBrk + tAcc) && dt < (tBrk + tAcc + tVel)) {
                    pos = oldPos + signM * (-dBrk + dAcc + constrains.maxVel * (dt - dBrk - tAcc))
                    vel = signM * constrains.maxVel;
                    acc = 0.0
                } else if (dt >= (dBrk + tAcc + tVel) && dt < (tBrk + tAcc + tVel + tDec)) {
                    pos = oldPos + signM * (-dBrk + dAcc + dVel + constrains.maxVel * (dt - tBrk - tAcc - tVel) - 0.5 * constrains.maxAcc * (dt - tBrk - tAcc) * (dt - tBrk - tAcc));
                    vel = signM * (constrains.maxVel - constrains.maxAcc * (dt - tBrk - tAcc - tVel));
                    acc = -signM * constrains.maxAcc
                } else {
                    pos = posRef;
                    vel = 0.0
                    acc = 0.0
                    isFinished = true;
                }
            }
            Shape.Tri -> {
                if (dt <= (tBrk + tAcc)) {
                    pos = oldPos + oldVel * dt + signM * 0.5 * constrains.maxAcc * dt * dt
                    vel = oldVel + signM * constrains.maxAcc * dt
                    acc = signM * constrains.maxAcc;
                } else if (dt > (tBrk + tAcc) && dt < (tBrk + tAcc + tDec)) {
                    pos = oldPos + signM * (-dBrk + dAcc + velSt * (dt - tBrk - tAcc) - 0.5 * constrains.maxAcc * (dt - tBrk - tAcc) * (dt - tBrk - tAcc));
                    vel = signM * (velSt - constrains.maxAcc * (dt - tBrk - tAcc));
                    acc = -signM * constrains.maxAcc;
                } else {
                    pos = posRef;
                    vel = 0.0;
                    acc = 0.0
                    isFinished = true;
                }
            }
        }
    }
}
