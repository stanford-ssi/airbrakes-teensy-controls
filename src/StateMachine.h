#pragma once

// Runs one tick of the flight state machine using the latest filtered AGL
// altitude. Mutates the global `state`, BrakeState, FlightState, and the
// status LED. Call once per main loop tick.
void updateStateMachine(float altitude);

// Clears the file-static debounce counters in StateMachine.cpp. Called by
// SOFT_RESET so a reset issued mid-flight does not carry stale burnout /
// apogee tallies into the next attempt.
void resetStateMachineCounters();
