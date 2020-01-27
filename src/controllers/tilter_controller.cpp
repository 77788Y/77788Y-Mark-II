#include "controllers/tilter_controller.hpp"
#include "subsystems/subsystems.hpp"

// constructor
TilterController::TilterController(std::shared_ptr<Tilter> tilter, std::shared_ptr<Transmission> transmission, std::shared_ptr<Lift> lift, double kp, double ki, double kd):
  m_tilter(tilter),
  m_transmission(transmission),
  m_lift(lift),
  m_controller(std::make_unique<IterativePosPIDController>(IterativeControllerFactory::posPID(kp, ki, kd))),
  m_enabled(false)
{

  // target
  m_controller->setTarget(Tilter::MAX_EXTENDED.convert(degree));

  // task
  m_task = std::make_unique<pros::Task>([=]() {

    while (true) {
      if (m_enabled && m_transmission->m_control_mutex.take(0)) {
        if (m_lift->m_control_mutex.take(0)) {
          std::cout << "hi" << std::endl;
          while (m_enabled) {

            m_tilter->update_angle();
            m_lift->update_angles();

            std::cout << tilter->get_angle().convert(degree) << "\t" << m_controller->getOutput() << std::endl;
            if (tilter->get_angle() >= Transmission::TILTER_EXTEND_THRESHOLD) {
              m_enabled = false;
              break;
            }

            lift->move_voltage(-4500);
            tilter->move_voltage(m_controller->step(tilter->get_angle().convert(degree)) * -12000);
            m_transmission->update();
            pros::delay(10);
          }
          std::cout << "bye" << std::endl;
          m_tilter->hold(1000);

          m_lift->m_control_mutex.give();
        }
        m_transmission->m_control_mutex.give();
      }
      pros::delay(10);
    }
  });
}

// enable controller
void TilterController::enable() {
  m_enabled = true;
}

// enable controller
void TilterController::disable() {
  m_enabled = false;
}
