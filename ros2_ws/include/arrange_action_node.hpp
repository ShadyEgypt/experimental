/**
 * @class ArrangeAction
 * @brief PlanSys2 action executor for arranging detected markers.
 *
 * This action:
 * - Reads detected ArUco marker data from YAML files
 * - Sorts markers deterministically
 * - Writes the arranged result to a shared YAML file
 *
 * It is used as part of a symbolic planning pipeline in PlanSys2.
 */
class ArrangeAction : public plansys2::ActionExecutorClient
{
public:
  /**
   * @brief Constructor.
   *
   * Initializes the action executor with the name "arrange"
   * and configures execution frequency.
   */
  ArrangeAction();

protected:
  /**
   * @brief Periodic execution callback.
   *
   * Performs the arrangement once and reports success or failure
   * to the PlanSys2 executor.
   */
  bool do_work() override;

private:
  /**
   * @brief Reads marker data, sorts it, and writes the arrangement.
   */
  void arrange_markers();
};
