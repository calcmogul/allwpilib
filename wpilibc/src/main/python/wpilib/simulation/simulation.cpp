
#include "semiwrap_init.wpilib.simulation._simulation.hpp"

void resetWpilibSimulationData() {}

SEMIWRAP_PYBIND11_MODULE(m) {
  initWrapper(m);

  m.def("_reset_wpilib_simulation_data", &resetWpilibSimulationData,
        release_gil());
}
