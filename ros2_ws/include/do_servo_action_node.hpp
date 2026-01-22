/**
 * @class DoServoAction
 * @brief PlanSys2 action executor for visual servoing control.
 *
 * This action:
 * - Performs visual servoing based on perceived target data
 * - Computes control commands to align the robot with a marker
 * - Continuously refines motion until alignment criteria are met
 *
 * It enables fine-grained motion control as part of the
 * PlanSys2 symbolic planning and execution pipeline.
 */
class DoServoAction : public plansys2::ActionExecutorClient
{
public:
  /**
   * @brief Constructor.
   *
   * Initializes the action executor with the name "do_servo"
   * and configures execution frequency.
   */
  DoServoAction();

protected:
  /**
   * @brief Periodic execution callback.
   *
   * Executes one step of the visual servoing loop and checks
   * convergence conditions to determine completion.
   *
   * @return true when servoing has completed
   */
  bool do_work() override;

private:
  /**
   * @brief Computes and applies servoing control commands.
   *
   * Uses perception feedback (e.g., marker pose or error)
   * to generate motion commands for precise alignment.
   */
  void perform_servoing();

  /**
   * @brief Checks whether servoing has converged.
   *
   * @return true if alignment accuracy is within tolerance
   */
  bool servoing_converged() const;
};
