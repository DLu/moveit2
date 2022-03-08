# Migration Strategery

This file should be deleted once all tasks are complete.

## Steps to Migrate
 * `moveit_setup_core_plugins`
   * [x] Start Screen
   * [x] Author Information
   * [x] Configuration Files
 * `moveit_setup_srdf_plugins`
   * [x] Self-Collisions
   * [x] Virtual Joints
   * [x] Planning Groups
     * BUG: Creating a new group does not properly create a new JointModelGroup
     * TODO: Need method for getting a list of OMPL Planners
   * [x] Robot Poses
     * TODO: Need to re-add sleeping code
   * [x] End Effectors
   * [x] Passive Joints
 * `moveit_setup_app_plugins`
   * [ ] ROS Control
   * [ ] Simulation
   * [x] Perception
      * TODO: Load default configuration
   * [ ] Bonus Content - Need to create a NEW setup step screen that allows users to configure what bonus launch files they want to be created, i.e. `demo.launch` and the like.
 * `moveit_setup_assistant`
   * TODO: Maybe add explicit SIGINT handler
   * TODO: Maybe add explicit ASyncSpinner equivalent / any spinning is needed (see SetupAssistantWidget::updateTimer)

### Instructions for migrating steps
 1. Move "Business Logic" out of `SETUP_STEP_widget.xpp` This is a more involved process, but a good place to start is to find all references to `config_data_` and move to `SETUP_STEP.xpp`

### Instructions for Adding Configs/Generated Files
 * Lots of additional useful code can be found in the `old_planner` folder.
 * Additional code removed from other classes can be found in `extra_*.txt` in this folder.

## Possible Enhancements
 * There's a common pattern in `GeneratedFile` subclasses which passes a reference to the parent object. This could be templated for reducing code duplication.
