  // gazebo_controllers.yaml ------------------------------------------------------------------
  file.file_name_ = "gazebo_controllers.yaml";
  file.rel_path_ = config_data_->appendPaths(config_path, file.file_name_);
  template_path = config_data_->appendPaths(config_data_->template_package_path_, file.rel_path_);
  file.description_ = "Configuration of Gazebo controllers";
  file.gen_func_ = std::bind(&ConfigurationFilesWidget::copyTemplate, this, template_path, std::placeholders::_1);

  // demo_gazebo.launch ------------------------------------------------------------------
  file.file_name_ = "demo_gazebo.launch";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Run a demo of MoveIt with Gazebo and Rviz";
  file.gen_func_ = std::bind(&ConfigurationFilesWidget::copyTemplate, this, template_path, std::placeholders::_1);

  // gazebo.launch ------------------------------------------------------------------
  file.file_name_ = "gazebo.launch";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, "gazebo.launch");
  file.description_ = "Gazebo launch file which also launches ros_controllers and sends robot urdf to param server, "
                      "then using gazebo_ros pkg the robot is spawned to Gazebo";
  file.gen_func_ = std::bind(&ConfigurationFilesWidget::copyTemplate, this, template_path, std::placeholders::_1);
