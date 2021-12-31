# Migration Strategery

This file should be deleted once all tasks are complete. Also delete `SETUP_STEP.{c|h}pp`

## Steps to Migrate
 * `moveit_setup_core_plugins`
   * [x] Start Screen
   * [x] Author Information
   * [x] Configuration Files
 * `moveit_setup_srdf_plugins`
   * [x] Self-Collisions
   * [x] Virtual Joints
   * [ ] Planning Groups
   * [ ] Robot Poses
   * [ ] End Effectors
   * [ ] Passive Joints
 * `moveit_setup_app_plugins`
   * [ ] ROS Control
   * [ ] Simulation
   * [x] Perception

### Instructions for migrating steps
 1. Move "Business Logic" out of `SETUP_STEP_widget.xpp` This is a more involved process, but a good place to start is to find all references to `config_data_` and move to `SETUP_STEP.xpp`

### Instructions for Adding Configs/Generated Files
 * Lots of additional useful code can be found in the `old_planner` folder.
 * Additional code removed from other classes can be found in `extra_*.txt` in this folder.

## Bugs
 * [ ] If you click on disabled steps in the navigation widget, the whole thing crashes

## Possible Enhancements
 * There's a common pattern in `GeneratedFile` subclasses which passes a reference to the parent object. This could be templated for reducing code duplication.
