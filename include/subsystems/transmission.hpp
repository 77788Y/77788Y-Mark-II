#include "subsystems/subsystem.hpp"
#include "state_machine.hpp"


/**
 * A struct storing the pose of the chassis.
 */
struct ChassisPose {
  QLength m_x; ///< X coordinate of chassis
  QLength m_y; ///< Y coordinate of chassis
  QAngle m_heading; ///< Orientation of chassis
  QLength m_encoder_dist_left; ///< Distance that the left encoder has travelled
  QLength m_encoder_dist_right; ///< Distance that the right encoder has travelled
  QLength m_encoder_dist_side; ///< Distance that the sideways encoder has travelled

  /**
   * Constructor
   * 
   * \param x
   *        X coordinate of chassis
   * \param y
   *        Y coordinate of chassis
   * \param heading
   *        Orientation of chassis
   * \param enc_l
   *        Distance that the left encoder has travelled
   * \param enc_r
   *        Distance that the right encoder has travelled
   * \param enc_s
   *        Distance that the sideways encoder has travelled
   */
  ChassisPose(QLength x, QLength y, QAngle heading, QLength enc_l = 0_in, QLength enc_r = 0_in, QLength enc_s = 0_in):
    m_x(x), m_y(y), m_heading(heading), m_encoder_dist_left(enc_l), m_encoder_dist_right(enc_r), m_encoder_dist_side(enc_s) {}

  /**
   * Addition operator overload
   * 
   * \param z
   *        A ChassisPose to add
   */
  ChassisPose operator+(ChassisPose& z) {
    QLength x = m_x + z.m_x;
    QLength y = m_y + z.m_y;
    QAngle heading = m_heading + z.m_heading;
    QLength enc_l = m_encoder_dist_left + z.m_encoder_dist_left;
    QLength enc_r = m_encoder_dist_right + z.m_encoder_dist_right;
    QLength enc_s = m_encoder_dist_side + z.m_encoder_dist_side;

    return ChassisPose(x, y, heading, enc_l, enc_r, enc_s);
  }

  /**
   * Subtraction operator overload
   * 
   * \param z
   *        A ChassisPose to add
   */
  ChassisPose operator-(ChassisPose& z) {
    QLength x = m_x - z.m_x;
    QLength y = m_y - z.m_y;
    QAngle heading = m_heading - z.m_heading;
    QLength enc_l = m_encoder_dist_left - z.m_encoder_dist_left;
    QLength enc_r = m_encoder_dist_right - z.m_encoder_dist_right;
    QLength enc_s = m_encoder_dist_side - z.m_encoder_dist_side;

    return ChassisPose(x, y, heading, enc_l, enc_r, enc_s);
  }
};

/**
 * A struct storing the derivative of the pose of the chassis.
 */
struct ChassisDeriv {
  QSpeed m_x; ///< X coordinate of chassis
  QSpeed m_y; ///< Y coordinate of chassis
  QAngularSpeed m_heading; ///< Orientation of chassis
  QSpeed m_encoder_dist_left; ///< Distance that the left encoder has travelled
  QSpeed m_encoder_dist_right; ///< Distance that the right encoder has travelled
  QSpeed m_encoder_dist_side; ///< Distance that the sideways encoder has travelled

  /**
   * Constructor
   * 
   * \param x
   *        X coordinate of chassis
   * \param y
   *        Y coordinate of chassis
   * \param heading
   *        Orientation of chassis
   * \param enc_l
   *        Distance that the left encoder has travelled
   * \param enc_r
   *        Distance that the right encoder has travelled
   * \param enc_s
   *        Distance that the sideways encoder has travelled
   */
  ChassisDeriv(QSpeed x, QSpeed y, QAngularSpeed heading, QSpeed enc_l = 0_mps, QSpeed enc_r = 0_mps, QSpeed enc_s = 0_mps):
    m_x(x), m_y(y), m_heading(heading), m_encoder_dist_left(enc_l), m_encoder_dist_right(enc_r), m_encoder_dist_side(enc_s) {}
};


/**
 * The controller of the transmission.
 * Manages the transmission's state machine and motors.
 * Should only be interfaced with by Chassis and Tilter friend classes.
 */
class Transmission {

  friend class Chassis;
  friend class Tilter;

private:

  /**
   * Motors associated with the transmission.
   */
  pros::Motor m_motor_left_direct; ///< The direct motor on the left of the chassis
  pros::Motor m_motor_right_direct; ///< The direct motor on the right of the chassis
  pros::Motor m_motor_left_transmission; ///< The shared motor on the left of the chassis
  pros::Motor m_motor_right_transmission; ///< The shared motor on the right of the chassis

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
   * The transmission's state machine.
   */
  StateMachine<StateType, BehaviorType> m_state_machine;

  /**
   * The desired voltages of the chassis.
   */
  int m_desired_chassis_voltage_left; ///< Desired voltage of the left chassis side
  int m_desired_chassis_voltage_right; ///< Desired voltage of the right chassis side

  /**
   * Set the desired state of the transmission.
   * 
   * \param state
   *        The desired state
   */
  void set_state(StateType state);
};


/**
 * The interface of the chassis.
 * Interfaces with the state machine in a Transmission object.
 */
class Chassis: private AbstractSubsystem<ChassisPose, ChassisDeriv> {

  friend class Transmission;

public:

  /**
   * Set the chassis motors to specified voltages.
   * Chassis is NOT garunteed to immediately (or ever) reach desired voltages.
   * The actual voltage depends on the Transmission object's Behavior.
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
   * The actual voltage depends on the Transmission object's Behavior.
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
  ChassisPose get_pose();

  /**
   * Tare the chassis' pose to a new pose.
   * 
   * \param new_pose
   *        The pose at which the chassis will now be
   */
  void tare_pose(ChassisPose new_pose);

  /**
   * Get the current pose derivative.
   * This is the speed of translation, rotation, etc.
   */
  ChassisDeriv get_deriv();

  /**
   * Update the chassis interface's pose calculation.
   * This will update the odometry algorithm, as well as the stored values of the sensors themselves.
   * Should always be run before acting on the chassis.
   */
  void update_pose();


private:

  /**
   * The encoders on the chassis' tracking wheels.
   */
  pros::ADIEncoder m_enc_left; ///< The encoder on the left tracking wheel
  pros::ADIEncoder m_enc_right; ///< The encoder on the right tracking wheel
  pros::ADIEncoder m_enc_side; ///< The encoder on the sideways tracking wheel

  /**
   * The reference pose of the chassis.
   * Acts as the "zero-point" from which the visible pose is calculated.
   */
  ChassisPose m_reference_pose;

  /**
   * The absolute pose of the chassis.
   * Does NOT account for m_reference_pose, but is relative to the creation of the subsystem.
   */
  ChassisPose m_absolute_pose;

  /**
   * The current pose of the chassis.
   * Relative to m_reference_pose.
   */
  ChassisPose m_pose;

  /**
   * The current derivative of the pose of the chassis.
   * Taring does not affect this value.
   */
  ChassisDeriv m_deriv;
};