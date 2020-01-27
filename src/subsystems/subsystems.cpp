#include "subsystems/subsystems.hpp"

namespace subsystems {

  // notify updater task
  void notify_updater(UpdaterNotification notification) {
    updater_task->notify_ext(notification, pros::E_NOTIFY_ACTION_OWRITE, nullptr);
  }

  // updater task
  std::shared_ptr<pros::Task> updater_task;

  // transmission
  std::shared_ptr<Transmission> transmission = std::make_shared<Transmission>(11, 20, 15, 16);
  auto odom = std::make_unique<Odom>(
    std::make_unique<ADIEncoder>('G', 'H', false),
    std::make_unique<ADIEncoder>('C', 'D', false),
    std::make_unique<ADIEncoder>('E', 'F', false),
    transmission->m_ime_left_direct,
    transmission->m_ime_right_direct,
    nullptr,
    8_in, 14_in, 0_in
  );

  // chassis
  std::shared_ptr<Chassis> chassis = std::make_shared<Chassis>(transmission, std::move(odom));

  // tilter
  std::shared_ptr<Tilter> tilter = std::make_shared<Tilter>(transmission);

  // intake
  std::shared_ptr<Intake> intake = std::make_shared<Intake>(12, 19);

  // lift
  std::shared_ptr<Lift> lift = std::make_shared<Lift>(1, 18);

  // initialize
  void init() {
    transmission->set_chassis(chassis);
    transmission->set_tilter(tilter);
    transmission->set_lift(lift);
    updater_task = std::make_shared<pros::Task>([]() {
      while (true) {

        update_poses();
        update_controllers();
        pros::delay(10);
      }
    });
  }

  // update poses
  void update_poses() {

    // transmission
    if (transmission->m_control_mutex.take(0)) {
      chassis->update_pose();
      tilter->update_angle();
      transmission->m_control_mutex.give();
    }

    // intake
    if (intake->m_control_mutex.take(0)) {
      intake->update_angles();
      intake->m_control_mutex.give();
    }

    // lift
    if (lift->m_control_mutex.take(0)) {
      lift->update_angles();
      lift->m_control_mutex.give();
    }
  }

  // update controllers
  void update_controllers() {

    // transmission
    if (transmission->m_control_mutex.take(0)) {
      transmission->update();
      transmission->m_control_mutex.give();
    }
  }
}