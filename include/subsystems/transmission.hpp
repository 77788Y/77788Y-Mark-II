#pragma once

#include "main.h"
#include <memory>

class Chassis;
class Tilter;

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
   * Globals.
   */
  static constexpr QAngle TILTER_RETRACT_THRESHOLD = 5_deg; ///< Tilter is considered retracted when behind this value.
  static constexpr QAngle TILTER_EXTEND_THRESHOLD = 5_deg;  ///< Tilter is considered extended when in front of this value.
  static constexpr double TILTER_HOLD_STRENGTH = 4000;      ///< This is the maximum voltage that will be applied to correct the tilter.

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
  Transmission(int8_t motor_left_direct, int8_t motor_right_direct, int8_t motor_left_shared, int8_t motor_right_shared);

  /**
   * IMEs associated with each motor.
   */
  std::shared_ptr<IntegratedEncoder> m_ime_left_direct;
  std::shared_ptr<IntegratedEncoder> m_ime_right_direct;
  std::shared_ptr<IntegratedEncoder> m_ime_left_shared;
  std::shared_ptr<IntegratedEncoder> m_ime_right_shared;

  /**
   * Control mutex.
   * Take this when controlling the transmission.
   * You must update the pose and internal controller manualls if taken.
   */
  pros::Mutex m_control_mutex;

  /**
   * Set the internal chassis reference.
   *
   * \param chassis
   *        A reference to the chassis
   */
  void set_chassis(std::shared_ptr<Chassis> chassis);

  /**
   * Set the internal tilter reference.
   *
   * \param tilter
   *        A reference to the tilter
   */
  void set_tilter(std::shared_ptr<Tilter> tilter);

  /**
   * Update the internal controller and state manager.
   */
  void update();

private:

  /**
   * Chassis and Tilter objects associated with this Transmission.
   */
  std::shared_ptr<Chassis> m_chassis;
  std::shared_ptr<Tilter> m_tilter;

  /**
   * Motors associated with the transmission.
   */
  const std::unique_ptr<Motor> m_motor_left_direct;  ///< The direct motor on the left of the chassis
  const std::unique_ptr<Motor> m_motor_right_direct; ///< The direct motor on the right of the chassis
  const std::unique_ptr<Motor> m_motor_left_shared;  ///< The shared motor on the left of the chassis
  const std::unique_ptr<Motor> m_motor_right_shared; ///< The shared motor on the right of the chassis

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
  State m_state;

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
  void set_state(State state);

  /**
   * PID controller.
   * Used to hold tilter in place when in HOLD state.
   */
  IterativePosPIDController m_hold_controller;
};
