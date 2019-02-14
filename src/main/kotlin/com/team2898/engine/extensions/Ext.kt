package com.team2898.engine.extensions

import edu.wpi.first.wpilibj.Encoder
import kotlinx.coroutines.Job
import kotlinx.coroutines.runBlocking
import org.jetbrains.annotations.TestOnly

fun Job.blockJoin() = runBlocking<Unit> { join() }

// Used for nullifying function returns
// (eg. we return a Job from AsyncLooper start() and stop() that we don't necessarily want to pass on if we're
// aliasing the function in a class)

fun Any.Unit() = Unit

operator fun Pair<Double, Double>.minus(other: Pair<Double, Double>) =
        Pair(first - other.first, second - other.second)
operator fun Pair<Double, Double>.times(num: Double) =
        Pair(first * num, second * num)

fun Pair<Any, Any>.get(index: Int) =
        if (index == 0) first else second
operator fun Pair<Int, Int>.get(index: Int) =
        if (index == 0) first else second
operator fun Pair<Double, Double>.get(index: Int) =
        if (index == 0) first else second

fun Pair<Any, Any>.l() = first
fun Pair<Any, Any>.r() = second

fun Pair<Double, Double>.avg() = (first + second) / 2


