#pragma once

namespace transmission {


  /**
   * Describe a way for the transmission to reconcile the chassis and tilter.
   */
  enum class BehaviorType {
    PASSIVE,          ///< transmission motor copies dedicated motor; tilter should remain stationary but this is not enforced
    HOLDING,          ///< tilter is held in place; drive speed may be modified
    RETRACTING,       ///< transmission motor is locked at full reverse speed regardless of what the direct motor does
    EXTENDING_FAST,   ///< transmission motor is locked at full forward speed regardless of what the direct motor does
    EXTENDING_SLOW    ///< transmission motor slowly extends with a controlled velocity and acceleration; drive speed may be modified
  };


  /**
   * Describe the sates that the transmission may be in.
   */
  enum class StateType {
    LOCKED,             ///< fully retracted and locked (PASSIVE behavior)
    WAITING_FOR_UNLOCK, ///< fully retracted and locked but desiring to be extended (PASSIVE behavior)
    WAITING_FOR_LOCK,   ///< fully retracted but not locked (HOLDING behavior)
    RETRACTING,         ///< retracting (RETRACTING behavior)
    DEPOSITING_FAST,    ///< first stage of deposit, at full speed (EXTENDING_FAST behavior)
    DEPOSITING_DECEL,   ///< second stage of deposit, decelerating to avoid flinging cubes (EXTENDING_SLOW behavior)
    DEPOSITED           ///< fully deposited but still extended (HOLDING behavior)
  };


  /**
   * Get the current behavior of the transmission.
   * 
   * \return The current behavior of the transmission
   */
  BehaviorType get_behavior();


  /**
   * Get the current state of the transmission.
   * 
   * \return The current state of the transmission
   */
  StateType get_state();


  /**
   * Set the desired voltage of each side of the chassis.
   * Actual voltage is determined by the current behavior.
   *
   * \param left
   *        The desired voltage of left side
   * \param right
   *        The desired voltage of left side
   */
  void move_voltage(int left, int right);


  /**
   * Set the desired voltage of both sides of the chassis.
   * Actual voltage is determined by the current behavior.
   *
   * \param val
   *        The desired voltage of both sides
   */
  void move_voltage(int val);


  /**
   * Begin stack depositing.
   * State will end at DEPOSITED.
   */
   void deposit_stack();


    /**
     * Begin to retract the tray.
     * State will end at LOCKED.
     */
   void retract_tray();
}