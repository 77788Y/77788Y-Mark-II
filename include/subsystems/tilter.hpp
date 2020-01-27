#pragma once

#include "lib/odom.hpp"
#include "subsystems/transmission.hpp"


class Tilter {

  friend class Transmission;

public:

static constexpr QAngle MAX_EXTENDED = 90_deg;

  /**
   * Constructor.
   */
  Tilter(std::shared_ptr<Transmission> transmission);

  /**
   * Set the voltage of both motors.
   * Will switch the transmission to LOCKED_PASSTHROUGH state.
   * 
   * \param val
   *        The desired voltage
   */
  void move_voltage(int val);


  /**
   * Hold the tray.
   * This will relieve all current controllers (EXTENDING, RETRACTING, LOCKED_PASSTHROUGH).
   * Will switch to HOLDING state unless locked, in which case the transmission will automatically switch to PASSIVE.
   */
  void hold(int bias = 0);


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
  void retract_passive();


  /**
   * Get the angle of the tilter's control arm.
   * Should be run after update_pose() for up-to-date values.
   * 
   * \return the current angle of the tilter's control arm.
   */
  QAngle get_angle();


  /**
   * Get the angular velocity of the control arm.
   * Should be run after update_pose() for up-to-date values.
   * 
   * \return the current angular velocity of the control arm
   */
  QAngularSpeed get_velocity();


  /**
   * Get the angular acceleration of the control arm.
   * Should be run after update_pose() for up-to-date values.
   * 
   * \return the current angular acceleration of the control arm
   */
  QAngularAcceleration get_acceleration();


  /**
   * Tare the angle of the tilter's control arm.
   * 
   * \param new_pos The angle at which the control arm will be
   */
  void tare_angle(QAngle new_pose);

  /**
   * Update the pose calculation.
   * Should always be run before acting on the tilter.
   */
  void update_angle();

private:

  /**
   * The Transmission that this Tilter object interacts with.
   */
  std::shared_ptr<Transmission> m_transmission;

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
   * A VelMath object.
   * Used to calculate velocity and acceleration of tilter.
   */
  VelMath velmath;

};