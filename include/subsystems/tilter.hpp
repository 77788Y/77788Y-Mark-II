#include "subsystems/subsystem.hpp"
#include "lib/odom.hpp"
#include "subsystems/transmission.hpp"


class Tilter: private AbstractSubsystem<QAngle, QAngularSpeed> {

  friend class Transmission;

public:
  /**
   * Set the voltage of both motors.
   * Will switch the transmission to LOCKED_PASSTHROUGH state.
   * 
   * \param val
   *        The desired voltage
   */
  void move_voltage(int val);


  /**
   * Extend the tray.
   * This is passive and will only happen when the chassis allows.
   * State will be set to EXTENDING.
   */
  void extend_passive();


  /**
   * Retract the tray.
   * This is passive and will only happen when the chassis allows.
   * State will be set to RETRACTING.
   */
  void retract_tray();


  /**
   * Get the angle of the tilter's control arm.
   * 
   * \return the current angle of the tilter's control arm.
   */
  QAngle get_pose();


  /**
   * Tare the angle of the tilter's control arm.
   * 
   * \param new_pos The angle at which the control arm will be
   */
  void tare_pose(QAngle new_pose);


  /**
   * Get the angular velocity of the control arm.
   * 
   * \return the current angular velocity of the control arm
   */
  QAngularSpeed get_deriv();

  /**
   * Update the pose calculation.
   * Should always be run before acting on the tilter.
   */
  void update_pose();

private:

  /**
   * The reference pose.
   * Acts as the "zero-point" from which the visible pose is calculated.
   */
  QAngle m_reference_pose;

  /**
   * The absolute pose.
   * Does NOT account for m_reference_pose, but is relative to the creation of the tilter.
   */
  QAngle m_absolute_pose;

  /**
   * The current pose.
   * Relative to m_reference_pose.
   */
  QAngle m_pose;

  /**
   * The current derivative of the pose of the tilter.
   * Taring does not affect this value.
   */
  QAngularSpeed m_deriv;

};