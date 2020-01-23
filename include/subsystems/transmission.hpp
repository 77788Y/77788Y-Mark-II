#pragma once

#include "main.h"
#include <memory>

/**
 * The controller of the transmission.
 * Manages the transmission's state machine and motors.
 * Should only be interfaced with by Chassis and Tilter friend classes.
 */
class Transmission {

  friend class Chassis;
  friend class Tilter;

  public:
  /**
   * Constructor.
   * 
   * \param mtr_direct_left
   *        The left motor directly connected to the chassis
   * \param mtr_direct_right
   *        The right motor directly connected to the chassis
   * \param mtr_shared_left
   *        The left motor shared between the chassis and the tilter
   * \param mtr_shared_right
   */
  Transmission(std::unique_ptr<Motor> mtr_direct_left, std::unique_ptr<Motor> mtr_direct_right, std::unique_ptr<Motor> mtr_shared_left, std::unique_ptr<Motor> mtr_shared_right);

private:

  /**
   * Motors associated with the transmission.
   */
  const std::unique_ptr<Motor> m_motor_left_direct;  ///< The direct motor on the left of the chassis
  const std::unique_ptr<Motor> m_motor_right_direct; ///< The direct motor on the right of the chassis
  const std::unique_ptr<Motor> m_motor_left_shared;  ///< The shared motor on the left of the chassis
  const std::unique_ptr<Motor> m_motor_right_shared; ///< The shared motor on the right of the chassis

  /**
   * IMEs associated with each motor.
   */
  std::shared_ptr<IntegratedEncoder> m_ime_left_direct;
  std::shared_ptr<IntegratedEncoder> m_ime_right_direct;
  std::shared_ptr<IntegratedEncoder> m_ime_left_shared;
  std::shared_ptr<IntegratedEncoder> m_ime_right_shared;

  /**
   * Describe a way for the transmission to reconcile the chassis and tilter.
   */
  enum class State {
    PASSIVE,            ///< transmission motor copies dedicated motor; tilter should remain stationary but this is not enforced
    HOLDING,            ///< tilter is held in place; drive speed may be modified
    RETRACTING,         ///< transmission motor is locked at full reverse speed regardless of what the direct motor does
    EXTENDING,          ///< transmission motor is locked at full forward speed regardless of the the direct motor does
    LOCKED_PASSTHROUGH  ///< chassis motors lock, and tilter gets full feedforward signal
  };

  /***
   * The current state of the transmission.
   */
  State state;

  /**
   * The desired voltages of the chassis.
   */
  int16_t m_desired_chassis_voltage_left;  ///< Desired voltage of the left chassis side
  int16_t m_desired_chassis_voltage_right; ///< Desired voltage of the right chassis side

  /**
   * The desired voltage of the tilter.
   * Only used when in the LOCKED_PASSTHROUGH state.
   */
  int16_t m_desired_tilter_voltage;

  /**
   * Set the desired state of the transmission.
   * 
   * \param state
   *        The desired state
   */
  void set_state(State);

  /**
   * Update the internal controller and state manager.
   * 
   * \param dt
   *        The time that has passed since the last update
   */
  void update(QTime dt);
};
