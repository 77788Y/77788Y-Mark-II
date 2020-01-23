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
    std::shared_ptr<ContinuousRotarySensor> enc_left,
    std::shared_ptr<ContinuousRotarySensor> enc_right,
    std::shared_ptr<ContinuousRotarySensor> enc_side,
    std::shared_ptr<ContinuousRotarySensor> enc_left_backup,
    std::shared_ptr<ContinuousRotarySensor> enc_right_backup,
    QLength track_width, QLength side_dist, QLength wheel_radius = 1.375_in
  );

  private:

  /**
   * The encoders used to calculate pose.
   */
  std::shared_ptr<ContinuousRotarySensor> m_enc_left;
  std::shared_ptr<ContinuousRotarySensor> m_enc_right;
  std::shared_ptr<ContinuousRotarySensor> m_enc_side;
  std::shared_ptr<ContinuousRotarySensor> m_enc_left_backup;
  std::shared_ptr<ContinuousRotarySensor> m_enc_right_backup;

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

  /**
   * Physical characteristics.
   */
  QLength m_track_width;
  QLength m_side_dist;
  QLength m_wheel_radius;
};