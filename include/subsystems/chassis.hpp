#include "subsystems/subsystem.hpp"
#include "lib/odom.hpp"
#include "subsystems/transmission.hpp"


/**
 * The interface of the chassis.
 * Interfaces with the state machine in a Transmission object.
 */
class Chassis: private AbstractSubsystem<Odom::ChassisPose, Odom::ChassisDeriv> {

  friend class Transmission;

public:

  
  Chassis();

  /**
   * Constructor.
   */
  Chassis(Transmission* transmission, Odom* odom);

  /**
   * Set the chassis motors to specified voltages.
   * Chassis is NOT garunteed to immediately (or ever) reach desired voltages.
   * The actual voltage depends on the Transmission object's state.
   * 
   * \param l
   *        The desired voltage of the left side of the chassis
   * \param r
   *        The desired voltage of the right side of the chassis
   */
  void move_voltage(int l, int r);

  /**
   * Set the chassis motors to specified voltage.
   * Chassis is NOT garunteed to immediately (or ever) reach desired voltages.
   * The actual voltage depends on the Transmission object's state.
   * 
   * \param val
   *        The desired voltage of both sides of the chassis
   */
  void move_voltage(int val);

  /**
   * Get the current pose of the chassis.
   * 
   * \return The pose of the chassis
   */
  Odom::ChassisPose get_pose();

  /**
   * Tare the chassis' pose to a new pose.
   * 
   * \param new_pose
   *        The pose at which the chassis will now be
   */
  void tare_pose(Odom::ChassisPose new_pose);

  /**
   * Get the current pose derivative.
   * This is the speed of translation, rotation, etc.
   */
  Odom::ChassisDeriv get_deriv();

  /**
   * Update the chassis interface's pose calculation.
   * This will update the Odom object.
   * Should always be run before acting on the chassis.
   */
  void update_pose();


private:

  /**
   * A reference to the Transmission this Chassis controls.
   */
  Transmission* transmission;

  /**
   * A reference to the Odom that controls this Chassis' pose.
   */
  Odom* odom;
};