
#include "semiwrap_init.wpilib.simulation._simulation.hpp"

#ifndef __FIRST_SYSTEMCORE__

namespace wpi::impl {
void ResetSmartDashboardInstance();
}  // namespace wpi::impl

namespace wpi::util::impl {
void ResetSendableRegistry();
}  // namespace wpi::util::impl

void resetWpilibSimulationData() {
  wpi::impl::ResetSmartDashboardInstance();
  wpi::util::impl::ResetSendableRegistry();
}

#else
void resetWpilibSimulationData() {}
#endif

SEMIWRAP_PYBIND11_MODULE(m) {
  initWrapper(m);

  m.def("_reset_wpilib_simulation_data", &resetWpilibSimulationData,
        release_gil());
}
