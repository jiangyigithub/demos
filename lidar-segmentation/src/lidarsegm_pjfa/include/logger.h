/**
 * A basic logger for project "lipe". 
 * 
 * It wraps the boost logger.
 * 
 * # Basic logging: 
 * Include this header and use one of the following functions:
 * 
 *     BOOST_LOG_TRIVIAL(trace) << "A trace severity message";
 *     BOOST_LOG_TRIVIAL(debug) << "A debug severity message";
 *     BOOST_LOG_TRIVIAL(info) << "An informational severity message";
 *     BOOST_LOG_TRIVIAL(warning) << "A warning severity message";
 *     BOOST_LOG_TRIVIAL(error) << "An error severity message";
 *     BOOST_LOG_TRIVIAL(fatal) << "A fatal severity message";
 * 
 * # Handling the log level:
 * - Instantiate the class.
 * - Use the SetSeverityLevel functions to set the log level.
 * - Use the IsDebug and GetSeverityLevel functions to check the current log level.
 * 
 * # build information:
 * If you include it into a shared library, add the folowing preprocessor directive into your CMakeList.txt:
 *     add_definitions(-DBOOST_LOG_DYN_LINK=1)
 * Otherwise you will have some runtime errors.
 * 
 * @todo add license information here. In the meantime this is the property of Robert Bosch Kft, all rights reserved, use your own risk, etc.
 * 
 * @author Péter Lakatos (CC-AD/ENG1-Bp)
 */

#ifndef LIPE_LOGGER_H_
#define LIPE_LOGGER_H_

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/algorithm/string.hpp>

#include <string>

namespace lipe 
{    
  namespace logging {

    /**
     * Enum for severity-level.
     * 
     * Aliasing it for code-readability.
     * Values are:
     * - trace
     * - debug
     * - info
     * - warning
     * - error
     * - fatal
     */
    typedef typename  boost::log::trivial::severity_level SeverityLevel;

    class Logger {

      public:

        /**
         * Default constructor for the logger.
         * 
         * @param severity_level_string Name of the severity level as a string.
         */
        Logger(const std::string& severity_level_string = "error"){
          SetSeverityLevel(severity_level_string);
        }

        /**
         * Setter for the severity level.
         * 
         * @param severity_level_string Name of the severity level as a string.
         */
        void SetSeverityLevel(const std::string& severity_level_string) {         
          SetSeverityLevel(StringToSeverityLevel(severity_level_string));
        }

        /**
         * Setter for the severity level.
         * 
         * @param severity_level Severity level as an enum.
         */
        void SetSeverityLevel(const SeverityLevel& severity_level) {

          severity_level_ = CheckSeverityLevelRange(severity_level);

          boost::log::core::get()->set_filter
          (
            boost::log::trivial::severity >= GetSeverityLevel()
          );
        }

        /**
         * Getter for the severity level.
         *  
         * @returns The current severity level as an enum.
         */
        SeverityLevel GetSeverityLevel() const {
          return severity_level_;
        }

        /**
         * Getter for the severity level.
         *  
         * @returns The name of the current severity level as a string.
         */
        std::string GetSeverityLevelAsString() const {
          return SeverityLevelToString(GetSeverityLevel());
        }

        /**
         * Checks if current severity level is at least debug.
         *  
         * @returns True if the current severity level is debug or trace, False otherwise.
         */
        bool IsDebug() const {
          return GetSeverityLevel() <= SeverityLevel::debug;
        }
        
      private:

        /**
         * Check if severity level enum is in range.
         *
         * Check if severity level enum is in range. 
         * If so, logs an error message and returns a default severity_value.
         * @param severity_level The severity level as an enum.
         * @returns If the severity_level is out of range, returns "error", else the severity level itself.
         */
        static SeverityLevel CheckSeverityLevelRange(const SeverityLevel& severity_level) {
          SeverityLevel result = severity_level;
          if(int(result) < int(SeverityLevel::trace))
          {
            const std::string error_msg = "SeverityLevel is out of range.";
            BOOST_LOG_TRIVIAL(error) << error_msg;
            result = SeverityLevel::error;
          }
          if(int(result) > int(SeverityLevel::fatal))
          {
            const std::string error_msg = "SeverityLevel is out of range.";
            BOOST_LOG_TRIVIAL(error) << error_msg;
            result = SeverityLevel::error;
          }

          return result;
        }

        /**
         * Converts severity level from string to enum.
         *  
         * Converts severity level from its name (inputted as a string) to the corresponding enum value.
         * Not case sensitive. 
         * Valid values:
         * - trace
         * - debug
         * - info
         * - warning
         * - error
         * - fatal
         * In case of invalid value, it throws an exception.
         * @param severity_level_string The name of the severity level as a string.
         * @returns The severity level as an enum.
         */
        static SeverityLevel StringToSeverityLevel(const std::string& severity_level_string){
                    
          SeverityLevel result = SeverityLevel::error;

          if( true == boost::iequals(severity_level_string, "trace")){
            result = SeverityLevel::trace;
          } else if( true == boost::iequals(severity_level_string, "debug")){
            result = SeverityLevel::debug;
          } else if( true == boost::iequals(severity_level_string, "info")){
            result = SeverityLevel::info;
          } else if( true == boost::iequals(severity_level_string, "warning")){
            result = SeverityLevel::warning;
          } else if( true == boost::iequals(severity_level_string, "error")){
            result = SeverityLevel::error;
          } else if( true == boost::iequals(severity_level_string, "fatal")){
            result = SeverityLevel::fatal;
          } else {
            const std::string error_msg = "StringToSeverityLevel argument is invalid.";
            BOOST_LOG_TRIVIAL(error) << error_msg;
            result = SeverityLevel::error;
          }

          return result;
        }

        /**
         * Converts severity level from enum to string.
         *
         * Converts severity level from its enum value to its name.
         * In case of valid input, output is one of the following strings:
         * - trace
         * - debug
         * - info
         * - warning
         * - error
         * - fatal
         * In case of invalid value, it throws an exception.
         * @param severity_level The severity level as an enum.
         * @returns The name of the severity level as a string
         */
        static std::string SeverityLevelToString(const SeverityLevel& severity_level){
                    
          std::string result = "undefined";

          if( SeverityLevel::trace == severity_level){
            result = "trace";
          } else if( SeverityLevel::debug == severity_level){
            result = "debug";
          } else if( SeverityLevel::info == severity_level){
            result = "info";
          } else if( SeverityLevel::warning == severity_level){
            result = "warning";
          } else if( SeverityLevel::error == severity_level){
            result = "error";
          } else if( SeverityLevel::fatal == severity_level){
            result = "fatal";
          } else {
            const std::string error_msg = "SeverityLevelToString argument is out of range.";
            BOOST_LOG_TRIVIAL(error) << error_msg;
            result = "out_of_range";
          }

          return result;
        }

        SeverityLevel   severity_level_;      /**< Storing the severity level. */
    };
  }
}

#endif      //LIPE_LOGGER_H_