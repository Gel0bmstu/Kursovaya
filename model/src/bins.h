#ifndef BINS_H_
#define BINS_H_

#include "namespace/bins_namespace.h"
#include "lib/quaternions/include/quaternion.h"

template <typename T = float>
class Bins {
  public:
    Bins();
    Bins(bins::BinsProgrammSettings& settings);
    ~Bins() {};      

    bins::Status GetSolution();
    bins::Status OpenLogFile();
    bins::Status CheckLogFile();

    bins::Status Start();

  private:
    // bins::Status Initialization_();
    bins::Status Installation_();
    bins::Status LogSolutionToFile_();

  private:
    bins::BinsProgrammSettings settings_;

    bins::EnviromentParmeters<T> params_;
    bins::SatelliteLinearVelocity<T> ssk_linear_vels_;
    bins::SatelliteLinearVelocity<T> nssk_linear_vels_;
    bins::SatelliteAngulatVelocityIncrements<T> ssk_ang_vels_inc_;
    bins::SatelliteAngulatVelocityIncrements<T> nssk_ang_vels_inc_;
    bins::OrientationAngles<T> orient_angls_;
};

#endif // BINS_H_
