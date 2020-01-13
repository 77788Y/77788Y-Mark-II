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
  virtual void move_voltage(int val);


  /**
   * Get the pose of the subsystem.
   * Should be extended by each subsystem with an applicable function.
   * 
   * \return the current pose of the subsystem
   */
  virtual TPose get_pose();


  /**
   * Get the derivative of the subsystem's pose.
   * Should be extended by each subsystem with an applicable function.
   * 
   * \return the current pose of the subsystem
   */
  virtual TDeriv get_deriv();

};