#include "controllers/pull_out_controler.hpp"
#include "subsystems/subsystems.hpp"

// constructor
PullOutController::PullOutController(std::shared_ptr<Tilter> tilter, std::shared_ptr<Chassis> chassis, std::shared_ptr<Transmission> transmission, std::shared_ptr<Intake> intake):
  m_tilter(tilter),
  m_chassis(chassis),
  m_transmission(transmission),
  m_intake(intake),
  m_enabled(false)
{

  // task
  m_task = std::make_unique<pros::Task>([=]() {

    while (true) {
      if (m_enabled && m_transmission->m_control_mutex.take(0)) {
        if (m_intake->m_control_mutex.take(0)) {
          m_tilter->hold(1000);
          m_intake->move_voltage(-3000);
          m_chassis->move_voltage(-6000, -6000);
          while (m_enabled) {

            tilter->update_angle();
            chassis->update_pose();
            m_transmission->update();
            pros::delay(10);
          }
          m_tilter->hold(0);
          m_chassis->move_voltage(0);
          m_intake->lock();
          m_intake->m_control_mutex.give();
        }
        m_transmission->m_control_mutex.give();
      }
      pros::delay(10);
    }
  });
}

// enable controller
void PullOutController::enable() {
  m_enabled = true;
}

// enable controller
void PullOutController::disable() {
  m_enabled = false;
}
