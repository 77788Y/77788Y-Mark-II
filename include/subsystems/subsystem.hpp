#include "main.h"


/**
 * An abstract subsystem.
 * Should be extended by all subsystems in the robot.
 * 
 * \tparam TPose
 *         The pose control type of the subsystem.
 *         This is the data type that all pose control functions should use.
 *         Should generally be an okapi unit (e.g. QAngle, QLength, etc.)
 * 
 * \tparam TDeriv
 *         The derivative control type of the subsystem.
 *         This is the data type that all derivative control functions should use.
 *         Should generally be an okapi unit (e.g. QAngularSpeed, QSpeed, etc.)
 */

template <class TPose, class TDeriv>
class AbstractSubsystem {

public:

  /**
   * Set the voltage of all motors.
   * Should be extended for each subsystem with an applicable function.
   * 
   * \param val
   *        The desired voltage
   */
  virtual void move_voltage(int val) = 0;


  /**
   * Get the pose of the subsystem.
   * Should be extended by each subsystem with an applicable function.
   * 
   * \return the current pose of the subsystem
   */
  virtual TPose get_pose() = 0;


  /**
   * Tare the subsystem's pose to a new pose.
   * 
   * \param new_pos The pose at which the subsystem will be
   */
  virtual void tare_pose(TPose new_pose) = 0;


  /**
   * Get the derivative of the subsystem's pose.
   * Should be extended by each subsystem with an applicable function.
   * 
   * \return the current pose of the subsystem
   */
  virtual TDeriv get_deriv() = 0;

  /**
   * Update the controller's pose calculation.
   * Should always be run before acting on the subsystem.
   */
  virtual void update_pose() = 0;

private:

  /**
   * The reference pose of the subsystem.
   * Acts as the "zero-point" from which the visible pose is calculated.
   */
  TPose m_reference_pose;

  /**
   * The absolute pose of the subsystem.
   * Does NOT account for m_reference_pose, but is relative to the creation of the subsystem.
   */
  TPose m_absolute_pose;

  /**
   * The current pose of the subystem.
   * Relative to m_reference_pose.
   */
  TPose m_pose;

  /**
   * The current derivative of the pose of the subsystem.
   * Taring does not affect this value.
   */
  TDeriv m_deriv;

};