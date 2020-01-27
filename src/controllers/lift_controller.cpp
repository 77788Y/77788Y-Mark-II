#include "controllers/lift_controller.hpp"

// constructor
LiftController::LiftController(std::shared_ptr<Lift> lift, std::shared_ptr<Intake> intake, double kp, double ki, double kd):
  m_lift(lift), m_intake(intake), m_controller(std::make_unique<IterativePosPIDController>(IterativeControllerFactory::posPID(kp, ki, kd)))
{

  m_controller->setTarget(m_actual_target.convert(degree));

  while (m_lift->m_control_mutex.take(TIMEOUT_MAX)) {
    while (true) {

      std::cout << std::get<2>(lift->get_angle()).convert(degree) << "\t" << m_controller->getOutput() << std::endl;
      lift->update_angles();
      lift->move_voltage(m_controller->step(std::get<2>(lift->get_angle()).convert(degree)) * 12000);
      pros::delay(10);

    }
    m_lift->m_control_mutex.give();
  }
}

void LiftController::set_target(QAngle target) {
  m_actual_target = target;
  if (lowered) lower();
  else raise();
}

void LiftController::lower() {
  m_controller->setTarget((m_actual_target - LOWER_BY).convert(degree));
  lowered = true;
}

void LiftController::raise() {
  m_controller->setTarget((m_actual_target).convert(degree));
  lowered = false;
}