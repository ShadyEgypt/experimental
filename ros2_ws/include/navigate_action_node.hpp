/**
 * @class NavigateAction
 * @brief PlanSys2 action executor for robot navigation.
 *
 * This action:
 * - Sends navigation goals to the Nav2 stack
 * - Monitors feedback and execution status
 * - Reports success or failure to the PlanSys2 executor
 *
 * It represents the motion-level realization of symbolic
 * navigation actions in the planning pipeline.
 */
class NavigateAction : public plansys2::ActionExecutorClient
{
public:
  /**
   * @brief Constructor.
   *
   * Initializes the action executor with the name "navigate"
   * and configures execution frequency.
   */
  NavigateAction();

protected:
  /**
   * @brief Periodic execution callback.
   *
   * Sends the navigation goal if not already sent and
   * checks execution progress until completion.
   *
   * @return true when navigation has finished
   */
  bool do_work() override;

private:
  /**
   * @brief Sends a navigation goal to the Nav2 action server.
   *
   * The goal is computed from symbolic parameters
   * provided by the PlanSys2 planner.
   */
  void send_navigation_goal();

  /**
   * @brief Checks the current navigation status.
   *
   * Evaluates feedback from Nav2 and determines whether
   * the robot has reached its destination.
   *
   * @return true if the goal has been reached successfully
   */
  bool navigation_completed() const;
};
