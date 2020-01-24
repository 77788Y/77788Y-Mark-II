#include "subsystems/subsystems.hpp"

namespace subsystems {

  // transmission
  auto transmission = std::make_shared<Transmission>(0, 0, 0, 0);
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


  void init() {
    transmission->set_chassis(chassis);
    transmission->set_tilter(tilter);
  }
}