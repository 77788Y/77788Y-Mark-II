#pragma once

#include "main.h"
#include "subsystems/lift.hpp"
#include "subsystems/intake.hpp"

/**
 * Lift controller.
 */
class LiftController {

  public:

  /**
   * Lift will lower by this much when the intake is running.
   */
  static constexpr QAngle LOWER_BY = 5_deg;

  /**
   * Constructor.
   * 
   * \param lift
   *        A reference to the lift being controlled
   * \param intake
   *        A reference to the intake being controlled
   */
  LiftController(std::shared_ptr<Lift> lift, std::shared_ptr<Intake> intake, double kp, double ki, double kd);


  /**
   * Enable the controller.
   */
  void set_target(QAngle target);

  bool lowered;

  void lower();

  void raise();

  private:

  /**
   * Should the controller be enabled?
   */
  QAngle m_actual_target = 0_deg;

  /**
   * Reference to the subsystems being controlled.
   */
  std::shared_ptr<Lift> m_lift;
  std::shared_ptr<Intake> m_intake;

  /**
   * Controller.
   */
  std::unique_ptr<IterativePosPIDController> m_controller;

  /**
   * The controller task.
   */
  std::unique_ptr<pros::Task> m_task;

};