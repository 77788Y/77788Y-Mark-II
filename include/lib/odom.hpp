#pragma once

#include "main.h"
#include <memory>

/**
 * Odom class.
 * Tracks the position of the robot in 2D coordinates using three tracking wheels.
 */
class Odom {

  friend class Chassis;

  /**
   * A struct storing the pose of the chassis.
   */
  struct ChassisPose {
    QLength m_x;                  ///< X coordinate of chassis
    QLength m_y;                  ///< Y coordinate of chassis
    QAngle m_heading;             ///< Orientation of chassis
    QLength m_encoder_dist_left;  ///< Distance that the left encoder has travelled
    QLength m_encoder_dist_right; ///< Distance that the right encoder has travelled
    QLength m_encoder_dist_side;  ///< Distance that the sideways encoder has travelled

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
    QSpeed m_x;                  ///< X coordinate of chassis
    QSpeed m_y;                  ///< Y coordinate of chassis
    QAngularSpeed m_heading;     ///< Orientation of chassis
    QSpeed m_encoder_dist_left;  ///< Distance that the left encoder has travelled
    QSpeed m_encoder_dist_right; ///< Distance that the right encoder has travelled
    QSpeed m_encoder_dist_side;  ///< Distance that the sideways encoder has travelled

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

  public:

  /**
   * Constructor.
   * 
   * \param enc_left
   *        The encoder associated with the left of the drive
   * \param enc_right
   *        The encoder associated with the right of the drive
   * \param enc_side
   *        The encoder associated with the sideways tracking wheel
   * \param enc_left_backup
   *        A second encoder associated with the left of the drive
   * \param enc_right_backup
   *        A second encoder associated with the right of the drive
   * \param track_width
   *        The distance between the left and right tracking wheels
   * \param side_wheel_dist
   *        The distance from the side wheel to the tracking center
   * \param wheel_radius
   *        The radius of the tracking wheels
   */
  Odom(
    std::unique_ptr<ContinuousRotarySensor> enc_left,
    std::unique_ptr<ContinuousRotarySensor> enc_right,
    std::unique_ptr<ContinuousRotarySensor> enc_side,
    std::shared_ptr<ContinuousRotarySensor> enc_left_backup = nullptr,
    std::shared_ptr<ContinuousRotarySensor> enc_right_backup = nullptr,
    std::unique_ptr<pros::Imu> imu = nullptr,
    QLength track_width = 16_in, QLength secondary_track_width = 16_in, QLength side_dist = 0_in, QLength wheel_radius = 1.375_in
  );

  /**
   * Update the odom calculations.
   * Shoul be run frequently; a 10ms interval is recommended.
   */
  void update();

  /**
   * Get the current pose of the robot.
   * Should be run after update() to ensure up-to-date calculations.
   * 
   * \return The current pose of the bot
   */
  ChassisPose* get_pose();

  /**
   * Get the rate of change of the current pose of the robot.
   * Should be run after update() to ensure up-to-date calculations.
   * 
   * \return The rate of change of the current pose of the bot
   */
  ChassisDeriv* get_speed();

  /**
   * Tare the pose so that the current pose reads as the value provided.
   * 
   * \param new_pose
   *        The pose that the current pose will be tared to
   */
  void tare(ChassisPose* new_pose);

  private:

  /**
   * The encoders used to calculate pose.
   */
  std::unique_ptr<ContinuousRotarySensor> m_enc_left;
  std::unique_ptr<ContinuousRotarySensor> m_enc_right;
  std::unique_ptr<ContinuousRotarySensor> m_enc_side;
  std::shared_ptr<ContinuousRotarySensor> m_enc_left_backup;
  std::shared_ptr<ContinuousRotarySensor> m_enc_right_backup;

  /**
   * An IMU used to supplement the encoders for theta calculation.
   */
    std::unique_ptr<pros::Imu> m_imu;

  /**
   * The reference pose of the chassis.
   * Acts as the "zero-point" from which the visible pose is calculated.
   */
  std::unique_ptr<ChassisPose> m_reference_pose;

  /**
   * The absolute pose of the chassis.
   * Does NOT account for m_reference_pose, but is relative to the creation of the subsystem.
   */
  std::unique_ptr<ChassisPose> m_absolute_pose;

  /**
   * The current pose of the chassis.
   * Relative to m_reference_pose.
   */
  std::unique_ptr<ChassisPose> m_pose;

  /**
   * The current derivative of the pose of the chassis.
   * Taring does not affect this value.
   */
  std::unique_ptr<ChassisDeriv> m_deriv;

  /**
   * Physical characteristics.
   */
  QLength m_track_width;
  QLength m_secondary_track_width;
  QLength m_side_dist;
  QLength m_wheel_radius;
};