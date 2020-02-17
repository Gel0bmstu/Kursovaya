#ifndef BINS_NAMESPACE_H_
#define BINS_NAMESPACE_H_

#include <stdbool.h>

#define cpr(a) std::cout << a << std::endl

namespace bins {

  /**
   * @brief Статус выполнения функций
   *        и открытия файлов
   * 
   */
  enum class Status {
    Error   = -1,
    Success = 0,
    Opened  = 2
  };

  /**
   * @brief Тип файла логирования:
   *        В "Solution" записывается решение алгоритма
   *        В "Info" записывается информация о работе программы
   */
  enum class FileType {
    Solution = 0,
    Info     = 1
  };

  /**
   * @brief Класс параметров работы алгоритма БИНС
   * 
   */
  class BinsProgrammSettings {
    public:
      /**
       * @brief Класс параметров работы алгоритма БИНС
       * 
       * @param sim_time Время симуляции [с]
       * @param samp_rate Частота дискретизации [Гц]
       * @param deb_mode Режим отладки [bool]
       * @param log_file Режим логирования решения алгоритма [bool]
       *                 в файл
       */
      BinsProgrammSettings(float sim_time = 3600,
                          float samp_rate = 400,
                          bool deb_mode = false,
                          bool log_file = true) :  
        smapling_rate(samp_rate), simulation_time(sim_time),
        debug_mode(deb_mode), file_log(log_file) {};
    
    private:
      float smapling_rate;
      float simulation_time;

      bool debug_mode;
      bool file_log; 
  };
  
  /**
   * @brief Класс хранения информации о параметрах
   *        окружающией среды
   */
  template <typename T>
  class EnviromentParmeters {
    public:
      /**
       * @brief Класс хранения информации о параметрах
       *        окружающией среды
       * 
       * @param g Ускорение свободного падения [м/с^2]
       * @param h Высота над уровнем моря [м]
       * @param u Скорость вращения Земля [рад/сек]
       * @param r Радиус Земли [м]
       */
      EnviromentParmeters(T g = 9.81, 
                          T h = 200, 
                          T u = 7.29e-5, 
                          T r = 6360000) :
        g_(g), h_(h), u_(u), r_(r) {};

    private:  
      T g_;
      T h_;
      T u_;
      T r_;
  };
  
  /**
   * @brief Класс хранения информации об углах
   *        ориентации осей ССК спутника отностилеьно НССК
   * 
   */
  template <typename T>
  class OrientationAngles {
    public:
      /**
       * @brief Класс хранения информации об углах
       *        ориентации осей ССК спутника отностилеьно НССК
       * 
       * @param psi Угол крена [рад]
       * @param teta Угол тангажа [рад]
       * @param gamma Угол курса [рад]
       */
      OrientationAngles(T psi = 0,
                        T teta = 0,
                        T gamma = 0) :
        psi_(psi), teta_(teta), gamma_(gamma) {};
    
    private:
      T psi_;
      T teta_;
      T gamma_;
  };

  /**
   * @brief Класс хранения информации о линйных
   *        скоростях спутника по осям СК
   */
  template <typename T>
  class SatelliteLinearVelocity {
    public:
      /**
       * @brief Класс хранения информации о линйных
       *        скоростях спутника по осям СК
       * 
       * @param x Скорость по оси X [м/с]
       * @param y Скорость по оси Y [м/с]
       * @param z Скорость по оси Z [м/с]
       */
      SatelliteLinearVelocity(T x = 0,
                              T y = 0,
                              T z = 0) : 
        x_(x), y_(y), z_(z) {};



    private:
      T x_;
      T y_;
      T z_;
  };
  
  /**
   * @brief Класс хранения информации об угловых
   *        приращениях спутника по осям СК
   * 
   */
  template <typename T>
  class SatelliteAngulatVelocityIncrements {
    public:
      /**
       * @brief Класс хранения информации об угловых
       *        приращениях спутника по осям СК
       * 
       * @param x Приращение скорости по оси X [м/с]
       * @param y Приращение скорости по оси Y [м/с]
       * @param z Приращение скорости по оси Z [м/с]
       */
      SatelliteAngulatVelocityIncrements(T x = 0,
                                         T y = 0,
                                         T z = 0) : 
        x_(x), y_(y), z_(z) {};

    private:
      T x_;
      T y_;
      T z_;
  };
  
}

#endif // BINS_NAMESPACE_H_