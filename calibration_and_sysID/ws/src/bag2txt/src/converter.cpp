#include "converter.hpp"

void Converter::_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{  
  for(uint i = 0; i < this->_N_JOINTS; i++)
  {
    rclcpp::Time t = msg->header.stamp;

    std::ofstream f;
    std::fstream reader;
    reader.open("config.txt");
		f.open("config.txt", std::ios_base::app);

    /* If the file is empty we add the header */
    if(reader.peek() == std::ifstream::traits_type::eof())
    {
      f << "Timestamp | " << " p1 |" << " p2 |" << " p3 |" << " p4 |"
                          << " v1 |" << " v2 |" << " v3 |" << " v4 |"
        << std::endl;
      f << " =========================================== " << std::endl;

      this->start = t;
    }
    reader.close();


		f << (t-this->start).nanoseconds() << " "; 
    for(uint i = 0; i < this->_N_JOINTS; i++)
      f << msg->position[i] << " ";
    
    for(uint i = 0; i < this->_N_JOINTS; i++)
      f << msg->velocity[i] << " ";
    
    f << std::endl;
		f.close();
  }
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<Converter>("/joint_states", 4));
    rclcpp::shutdown();
    return 0;
}
