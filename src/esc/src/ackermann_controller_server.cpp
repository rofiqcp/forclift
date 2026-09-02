#include <rclcpp/rclcpp.hpp>
#include <nav2_controller/controller_server.hpp>

#include <memory>

// Fungsi: Menjalankan ControllerServer Nav2 standar dengan satu parameter MPPI
// Humble dipre-declare sebelum lifecycle configure/activate. MPPI Humble
// mendaftarkan callback dinamis sebelum mencoba mendeklarasikan
// "controller_server.verbose"; urutan upstream itu dapat menghasilkan satu
// warning palsu saat aktivasi. Pre-declare di wrapper ini menghilangkan warning
// tanpa memodifikasi algoritma ControllerServer atau MPPI.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_controller::ControllerServer>();

  if (!node->has_parameter("controller_server.verbose")) {
    node->declare_parameter<bool>("controller_server.verbose", false);
  }

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
