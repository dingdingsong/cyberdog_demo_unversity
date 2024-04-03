#include <memory>
#include "cyberdog_example/cyberdog_example.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    std::shared_ptr<cyberdog_example::CyberdogExample> cyberdog_example =
        std::make_shared<cyberdog_example::CyberdogExample>("cyberdog_example");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(cyberdog_example);
    executor.spin();
    return 0;
}