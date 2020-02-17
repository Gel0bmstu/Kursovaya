#include "bins.h"
#include "namespace/bins_namespace.h"
#include "lib/quaternions/include/quaternion.h"

template <typename T>
Bins<T>::Bins() {}

template <typename T>
Bins<T>::Bins(bins::BinsProgrammSettings& settings) : Bins::settings_(std::move(settings)) {}

template <typename T>
bins::Status Bins<T>::Start() {
  bins::Status status;
  
  status = Bins<T>::OpenLogFile();
  if (status == bins::Status::Error) {
    
  }

  status = Bins<T>::SetInitialConditions();

}

template <typename T>
bins::Status Bins<T>::GetSolution() {

  return bins::Status::Success;
}

// bins::Status Initialization_() {
//   return bins::Status::Success;
// }
