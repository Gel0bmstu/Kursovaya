#include <iostream>
#include <ctime>
#include <string.h>

#include "logger/logger.h"
#include "namespace/bins_namespace.h"

// Logger::~Logger() {
//   delete[] this->file_name_;

//   this->CloseLogFile();
// };

// Logger::Logger() {
//   char* date_and_time = Logger::GetCurrentDateAndTime_();
//   this->file_name_ = strcat("logs/log_", date_and_time);

//   std::cout << this->file_name_ << std::endl;

//   this->log_stream_.open(this->file_name_, std::fstream::in | std::fstream::out | std::fstream::app);

//   if (!this->log_stream_.is_open()) {
//     std::cerr << __FILE__ << ":" << __LINE__ << " [ERR] Unable to open log file." << std::endl; 
//   }

// };

// char* Logger::GetCurrentDateAndTime_() {
//   time_t t = time(NULL);
//   struct tm tm = *localtime(&t);

//   char* result = (char*)malloc(128 * sizeof(char));

//   sprintf(result, "%d-%d-%d %d:%d:%d", 
//           tm.tm_year + 1900,
//           tm.tm_mon + 1,
//           tm.tm_mday,
//           tm.tm_hour,
//           tm.tm_min,
//           tm.tm_sec);

//   return result;
// };

bins::Status Logger::CloseLogFile() {

  if (this->log_stream_.is_open()) {
    this->log_stream_.close();
  }
  
  return bins::Status::Success;
};