#pragma once

namespace transmission {


  // behavior states that the transmission may be in
  enum class Behavior {
    PASSIVE,          // transmission motor copies dedicated motor; tilter should remain stationary but this is not enforced
    HOLDING,          // tilter is held in place; drive speed may be modified
    RETRACTING,       // transmission motor is locked at full reverse speed regardless of what the direct motor does
    EXTENDING_FAST,   // transmission motor is locked at full forward speed regardless of what the direct motor does
    EXTENDING_SLOW    // transmission motor slowly extends with a controlled velocity and acceleration; drive speed may be modified
  };


  // states that the transmission may be in
  enum class State {
    LOCKED,             // fully retracted and locked (PASSIVE behavior)
    WAITING_FOR_LOCK,   // fully retracted but not locked (HOLDING behavior)
    RETRACTING,         // retracting (RETRACTING behavior)
    DEPOSITING_FAST,    // first stage of deposit, at full speed (EXTENDING_FAST behavior)
    DEPOSITING_DECEL,   // second stage of deposit, decelerating to avoid flinging cubes (EXTENDING_SLOW behavior)
    DEPOSITED           // fully deposited but still extended (HOLDING behavior)
  };


  // get the current behavior of the transmission
  Behavior get_behavior();

  // get the current state of the transmission
  State get_state();
}