#ifndef BINS_LOGGER_H_
#define BINS_LOGGER_H_

#include <cstring>
#include <fstream>
#include <iostream>

#include "namespace/bins_namespace.h"

/**
 * @brief Класс логгера:
 *        Записывает решения алгоритма/дебаг
 *        информацию о работе программы в файл.
 * 
 */
class Logger {
  public:
    Logger() {

      const char* date_and_time = this->GetCurrentDateAndTime_();

      cpr(date_and_time);
      this->file_name_ = new char[64];
      strcpy(this->file_name_, "logs/log_");
      strcat(this->file_name_, date_and_time);

      std::cout << this->file_name_ << std::endl;

      this->log_stream_.open(this->file_name_, std::fstream::in | std::fstream::out | std::fstream::app);

      if (!this->log_stream_.is_open()) {
        std::cerr << __FILE__ << ":" << __LINE__ << " [ERR] Unable to open log file." << std::endl; 
      }
    };
    ~Logger() {};

    // bins::Status OpenLogFile(char* adress_to_file);
    bins::Status Log(bins::FileType f, char* str);
    bins::Status CloseLogFile();

  private:
    char* GetCurrentDateAndTime_() {
        time_t t = time(NULL);
        struct tm tm = *localtime(&t);

        char* result = (char*)malloc(50 * sizeof(char));

        std::cout << "get 1" << std::endl;

        sprintf(result, "%d-%d-%d_%d:%d:%d", 
                tm.tm_year + 1900,
                tm.tm_mon + 1,
                tm.tm_mday,
                tm.tm_hour,
                tm.tm_min,
                tm.tm_sec);

        std::cout << "get 2" << std::endl;

        return result;
    };

  private:
    char* file_name_;
    std::ofstream log_stream_;
};

#endif // BINS_LOGGER_H_