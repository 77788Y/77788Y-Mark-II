#pragma once

#include "lib/odom.hpp"
#include "subsystems/transmission.hpp"


class Intake {

public:

  /**
   * Constructor.
   * 
   * \param port_l
   *        The port of the left roller
   * \param port_r
   *        The port of the right roller
   */
  Intake(int8_t port_l, int8_t port_r);

  /**
   * Control mutex.
   * Take this when controlling the intake.
   * You must update the pose and internal controller manualls if taken.
   */
  pros::Mutex m_control_mutex;

  /**
   * Set the voltage of both motors.
   * 
   * \param val
   *        The desired voltage
   */
  void move_voltage(int val);


  /**
   * Lock both intake motors.
   * Overriden by move_voltage().
   */
  void lock();


  /**
   * Get the angle of the rollers.
   * Should be run after update_pose() for up-to-date values.
   * 
   * \return
   *          The current angle of the left motor,
   *          The current angle of the right motor,
   *          The mean of the motors' angles
   */
  std::tuple<QAngle, QAngle, QAngle> get_angle();


  /**
   * Get the angular velocities of the rollers.
   * Should be run after update_pose() for up-to-date values.
   * 
   * \return
   *          The current anglular velocity of the left motor,
   *          The current anglular velocity of the right motor,
   *          The mean of the motors' anglular velocities
   */
  std::tuple<QAngularSpeed, QAngularSpeed, QAngularSpeed> get_velocity();


  /**
   * Get the angular accelerations of the rollers.
   * Should be run after update_pose() for up-to-date values.
   * 
   * \return
   *          The current anglular acceleration of the left motor,
   *          The current anglular acceleration of the right motor,
   *          The mean of the motors' anglular accelerations
   */
  std::tuple<QAngularAcceleration, QAngularAcceleration, QAngularAcceleration> get_acceleration();


  /**
   * Tare the angles of the rollers.
   * 
   * \param left
   *        The new angle of the left roller
   * \param right
   *        The new angle of the right roller
   */
  void tare_angles(QAngle left, QAngle right);

  /**
   * Update the pose calculation.
   * Should always be run before acting on the tilter.
   */
  void update_angles();

private:

  /**
   * Motors associated with the intake
   */
  std::unique_ptr<Motor> m_motor_left;  ///< The motor on the left roller
  std::unique_ptr<Motor> m_motor_right; ///< The motor on the right roller

  /**
   * The reference pose.
   * Acts as the "zero-point" from which the visible pose is calculated.
   */
  QAngle m_reference_pose_left;
  QAngle m_reference_pose_right;

  /**
   * The absolute pose.
   * Does NOT account for m_reference_pose, but is relative to the creation of the tilter.
   */
  QAngle m_absolute_pose_left;
  QAngle m_absolute_pose_right;

  /**
   * The current pose.
   * Relative to m_reference_pose.
   */
  QAngle m_pose_left;
  QAngle m_pose_right;

  /**
   * A VelMath object.
   * Used to calculate velocity and acceleration of rollers.
   */
  VelMath velmath_left;
  VelMath velmath_right;
};