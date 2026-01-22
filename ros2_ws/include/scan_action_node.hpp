/**
 * @class ScanAction
 * @brief PlanSys2 action executor for scanning ArUco markers.
 *
 * This action:
 * - Triggers a perception routine to detect ArUco markers
 * - Retrieves marker IDs and poses from sensor data
 * - Stores scan results into YAML files for later planning stages
 *
 * It serves as the perception entry point in the PlanSys2 symbolic
 * planning pipeline.
 */
class ScanAction : public plansys2::ActionExecutorClient
{
public:
  /**
   * @brief Constructor.
   *
   * Initializes the action executor with the name "scan"
   * and configures its execution frequency.
   */
  ScanAction();

protected:
  /**
   * @brief Periodic execution callback.
   *
   * Executes the scanning procedure once, saves detected markers,
   * and reports the result to the PlanSys2 executor.
   *
   * @return true when the action finishes (success or failure)
   */
  bool do_work() override;

private:
  /**
   * @brief Performs the marker scanning procedure.
   *
   * Collects detected marker data and serializes it into
   * structured YAML files for downstream actions.
   */
  void scan_markers();

  /**
   * @brief Writes scan results to a YAML file.
   *
   * @param marker_id Detected marker identifier
   * @param pose Marker pose in the map frame
   */
  void write_marker_yaml(int marker_id, const geometry_msgs::msg::Pose & pose);
};
