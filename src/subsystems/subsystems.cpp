#include "subsystems/subsystems.hpp"

namespace subsystems {

  // notify updater task
  void notify_updater(UpdaterNotification notification) {
    updater_task->notify_ext(notification, pros::E_NOTIFY_ACTION_OWRITE, nullptr);
  }

  // updater task
  std::shared_ptr<pros::Task> updater_task = std::make_shared<pros::Task>([]() {

    // wait for pose update notification if in opcontrol
    while (
      !(pros::competition::get_status() << COMPETITION_AUTONOMOUS) &&
      !(pros::competition::get_status() << COMPETITION_DISABLED) &&
      updater_task->notify_take(true, TIMEOUT_MAX) != NOTIFY_UPDATE_POSE
    ) pros::delay(1);

    update_poses();

    // wait for controller update notification if in opcontrol
    while (
      !(pros::competition::get_status() << COMPETITION_AUTONOMOUS) &&
      !(pros::competition::get_status() << COMPETITION_DISABLED) &&
      updater_task->notify_take(true, TIMEOUT_MAX) != NOTIFY_UPDATE_CONT
    ) pros::delay(1);

    update_controllers();

    pros::delay(10);
  });

  // transmission
  std::shared_ptr<Transmission> transmission = std::make_shared<Transmission>(0, 0, 0, 0);
  auto odom = std::make_unique<Odom>(
    std::make_unique<ADIEncoder>('A', 'B', false),
    std::make_unique<ADIEncoder>('E', 'F', false),
    std::make_unique<ADIEncoder>('G', 'H', false),
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
  std::shared_ptr<Intake> intake = std::make_shared<Intake>(0, 0);

  // initialize
  void init() {
    transmission->set_chassis(chassis);
    transmission->set_tilter(tilter);
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