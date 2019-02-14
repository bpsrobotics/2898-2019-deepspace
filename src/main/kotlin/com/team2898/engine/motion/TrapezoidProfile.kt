package com.team2898.engine.motion

import edu.wpi.first.wpilibj.Timer
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sign
import kotlin.math.sqrt


data class Constrains(val maxVel: Double, val maxAcc: Double)
data class ProfilePos(var position: Double, var velocity: Double)

class TrapezoidProfile(val constrains: Constrains) {

    var direction = 0
    var cutoffBegin = 0.0
    var cutoffDistBegin = 0.0
    var cutoffEnd = 0.0
    var cutoffDistEnd = 0.0

    var fullTrapDist = 0.0
    var accTime = 0.0

    var fullSpeedDist = 0.0


    var endAcc = 0.0
    var endFullSpeed = 0.0
    var endDec = 0.0

    var startTime = 0.0

    var initial = ProfilePos(0.0, 0.0)
    var final = initial

    fun updateState(i: ProfilePos, f: ProfilePos) {

        initial = i
        final = f
        direction = if (shouldFlipAcc()) -1 else 1
        initial = direct(initial)
        final = direct(final)

        cutoffBegin = initial.velocity / constrains.maxAcc
        cutoffDistBegin = cutoffBegin * cutoffBegin * constrains.maxAcc / 2.0
        cutoffEnd = final.velocity / constrains.maxAcc
        cutoffDistEnd = cutoffEnd * cutoffEnd * constrains.maxAcc / 2.0

        fullTrapDist =
            cutoffDistBegin + (final.position - initial.position) + cutoffDistEnd
        accTime = constrains.maxVel / constrains.maxAcc

        fullSpeedDist =
            fullTrapDist - accTime * accTime * constrains.maxAcc

        if (fullSpeedDist < 0.0) {
            accTime =
                    sqrt(fullSpeedDist / constrains.maxAcc)
            fullSpeedDist = 0.0
        }

        endAcc = accTime - cutoffBegin
        endFullSpeed = endAcc + fullSpeedDist / constrains.maxVel
        endDec = endFullSpeed + accTime - cutoffEnd
        startTime = Timer.getFPGATimestamp()
    }

    fun calc(t: Double = Timer.getFPGATimestamp()): ProfilePos {
        val time = startTime - t
        var result: ProfilePos = initial

        if (time < endAcc) {
            result.velocity += time * constrains.maxAcc
            result.position +=
                    (initial.velocity + time * constrains.maxAcc / 2.0) * time
        } else if (time < endFullSpeed) {
            result.velocity = constrains.maxVel
            result.position +=
                    (initial.velocity + endAcc * constrains.maxAcc / 2.0) * endAcc + constrains.maxVel * (t - endAcc)
        } else if (t <= endDec) {
            result.velocity =
                    final.velocity + (endDec - time) * constrains.maxAcc
            val timeLeft = endDec - time
            result.position =
                    final.position - (final.velocity + timeLeft * constrains.maxAcc / 2.0) * timeLeft
        } else {
            result = final
        }

        return direct(result)
    }

    fun direct(prof: ProfilePos): ProfilePos {
        val result = prof
        result.position *= this.direction
        result.velocity *= this.direction
        return result
    }


    fun shouldFlipAcc(c: Constrains = constrains,
                      i: ProfilePos = initial,
                      f: ProfilePos = final): Boolean {
        val velChange = f.velocity - i.velocity
        val distChange = f.position - i.position

        val time = abs(velChange) / c.maxAcc
        val shouldFlip = time * (velChange / 2 + i.velocity) > distChange
        return shouldFlip
    }
}


