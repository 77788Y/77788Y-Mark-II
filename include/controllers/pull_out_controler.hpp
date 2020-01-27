#pragma once

#include "main.h"
#include "subsystems/tilter.hpp"
#include "subsystems/chassis.hpp"
#include "subsystems/transmission.hpp"
#include "subsystems/intake.hpp"

/**
 * Tilter controller.
 * Use when depositing.
 */
class PullOutController {

  public:

  /**
   * Constructor.
   * 
   * \param tilter
   *        A reference to the tilter being controlled
   * \param chassis
   *        A reference to the chassis being controlled
   * \param transmission
   *        A reference to the transmission the tilter belongs to
   * \param intake
   *        A reference to the intake being controlled
   */
  PullOutController(std::shared_ptr<Tilter> tilter, std::shared_ptr<Chassis> chassis, std::shared_ptr<Transmission> transmission, std::shared_ptr<Intake> intake);


  /**
   * Enable the controller.
   */
  void enable();

  /**
   * Disable the controller.
   */
  void disable();

  private:

  /**
   * Should the controller be enabled?
   */
  bool m_enabled;

  /**
   * Reference to the subsystems being controlled.
   */
  std::shared_ptr<Tilter> m_tilter;
  std::shared_ptr<Chassis> m_chassis;
  std::shared_ptr<Transmission> m_transmission;
  std::shared_ptr<Intake> m_intake;

  /**
   * The controller task.
   */
  std::unique_ptr<pros::Task> m_task;

};