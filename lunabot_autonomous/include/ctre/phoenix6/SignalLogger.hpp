/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
#pragma once

#include "ctre/phoenix/StatusCodes.h"
#include <units/time.h>
#include <array>
#include <vector>

namespace ctre {
namespace phoenix6 {

/**
 * \brief Static class for controlling the Phoenix 6 signal logger.
 *
 * This logs all the signals from the CAN buses into .hoot files. Each file name starts with the
 * CANivore serial number or "rio" for the roboRIO CAN bus, followed by the timestamp. In the
 * header of a hoot file, the CANivore name and firmware version are logged in plain text.
 *
 * During an FRC match, the log file will be renamed to include the event name, match type, and
 * match number at the start of the file name. The match type will be 'P' for practice matches,
 * 'Q' for qualification matches, and 'E' for elimination matches.
 */
class SignalLogger
{
public:
    /**
     * \brief Sets the destination for logging, restarting logger if the path changed.
     *
     * If this is not called or the path is left empty, the default path will be used. The
     * default path on the roboRIO is a logs folder on the first USB flash drive found, or
     * /home/lvuser/logs if none is available. The default path on all other platforms is
     * a logs folder in the current working directory.
     *
     * Typical use for this routine is to use a removable USB flash drive for logging.
     *
     * \param path Folder path for the log files; path must exist
     * \returns Status of setting the path and restarting the log
     */
    static ctre::phoenix::StatusCode SetPath(const char *path);
    /**
     * \brief Starts logging status signals. Starts regardless of auto logging status.
     *
     * If using a roboRIO 1, we recommend setting the logging path to an external drive
     * using #SetPath to avoid running out of internal storage space.
     *
     * \details If auto logging is enabled, the log will be stopped at the end of the match.
     *
     * \returns Status of starting the logger
     */
    static ctre::phoenix::StatusCode Start();
    /**
     * \brief Stops logging status signals. Stops regardless of auto logging status.
     *
     * \returns Status of stopping the logger
     */
    static ctre::phoenix::StatusCode Stop();
    /**
     * \brief Enables or disables auto logging.
     *
     * Auto logging is only supported on the roboRIO. When auto logging is enabled,
     * logging is started at the beginning of an FRC match and stopped at the end.
     *
     * \param enable Whether to enable auto logging
     * \returns Status of auto logging enable/disable
     */
    static ctre::phoenix::StatusCode EnableAutoLogging(bool enable);

    /**
     * \brief Writes the raw data bytes to the log file. The data cannot exceed 64 bytes.
     * 
     * \param name Name of the signal
     * \param data Raw data bytes
     * \param size Size of the raw data (in bytes)
     * \param latencySeconds Latency of the signal in seconds; this value is subtracted
     *                       from the current time to get the timestamp written to the log
     * \returns Status of writing the data
     */
    static ctre::phoenix::StatusCode WriteRaw(std::string_view name, uint8_t const *data, uint8_t size, units::time::second_t latencySeconds = 0_s)
    {
        return WriteRaw_Impl(name, data, size, latencySeconds.value());
    }
    /**
     * \brief Writes the boolean to the log file.
     * 
     * \param name Name of the signal
     * \param value Value to write
     * \param latencySeconds Latency of the signal in seconds; this value is subtracted
     *                       from the current time to get the timestamp written to the log
     * \returns Status of writing the data
     */
    static ctre::phoenix::StatusCode WriteBoolean(std::string_view name, bool value, units::time::second_t latencySeconds = 0_s)
    {
        return WriteBoolean_Impl(name, value, latencySeconds.value());
    }
    /**
     * \brief Writes the integer to the log file.
     * 
     * \param name Name of the signal
     * \param value Value to write
     * \param units Units of the signal
     * \param latencySeconds Latency of the signal in seconds; this value is subtracted
     *                       from the current time to get the timestamp written to the log
     * \returns Status of writing the data
     */
    static ctre::phoenix::StatusCode WriteInteger(std::string_view name, int64_t value, std::string_view units = "", units::time::second_t latencySeconds = 0_s)
    {
        return WriteInteger_Impl(name, value, units, latencySeconds.value());
    }
    /**
     * \brief Writes the float to the log file.
     * 
     * \param name Name of the signal
     * \param value Value to write
     * \param units Units of the signal
     * \param latencySeconds Latency of the signal in seconds; this value is subtracted
     *                       from the current time to get the timestamp written to the log
     * \returns Status of writing the data
     */
    static ctre::phoenix::StatusCode WriteFloat(std::string_view name, float value, std::string_view units = "", units::time::second_t latencySeconds = 0_s)
    {
        return WriteFloat_Impl(name, value, units, latencySeconds.value());
    }
    /**
     * \brief Writes the double to the log file.
     * 
     * \param name Name of the signal
     * \param value Value to write
     * \param units Units of the signal
     * \param latencySeconds Latency of the signal in seconds; this value is subtracted
     *                       from the current time to get the timestamp written to the log
     * \returns Status of writing the data
     */
    static ctre::phoenix::StatusCode WriteDouble(std::string_view name, double value, std::string_view units = "", units::time::second_t latencySeconds = 0_s)
    {
        return WriteDouble_Impl(name, value, units, latencySeconds.value());
    }
    /**
     * \brief Writes the string to the log file. The string cannot exceed 64 characters.
     * 
     * \param name Name of the signal
     * \param value Value to write
     * \param latencySeconds Latency of the signal in seconds; this value is subtracted
     *                       from the current time to get the timestamp written to the log
     * \returns Status of writing the data
     */
    static ctre::phoenix::StatusCode WriteString(std::string_view name, std::string_view value, units::time::second_t latencySeconds = 0_s)
    {
        return WriteString_Impl(name, value, latencySeconds.value());
    }

    /**
     * \brief Writes the unit value to the log file.
     * 
     * \param name Name of the signal
     * \param value Value to write
     * \param latencySeconds Latency of the signal in seconds; this value is subtracted
     *                       from the current time to get the timestamp written to the log
     * \returns Status of writing the data
     */
    template <typename U, typename = std::enable_if_t<units::traits::is_unit_t_v<U>>>
    static ctre::phoenix::StatusCode WriteValue(std::string_view name, U value, units::time::second_t latencySeconds = 0_s)
    {
        return WriteDouble(name, value.value(), units::abbreviation(value), latencySeconds);
    }

    /**
     * \brief Writes the array of booleans to the log file. The array cannot exceed 64 elements.
     * 
     * \param name Name of the signal
     * \param values Array of values to write
     * \param latencySeconds Latency of the signal in seconds; this value is subtracted
     *                       from the current time to get the timestamp written to the log
     * \returns Status of writing the data
     */
    template <size_t N, typename = std::enable_if_t<(N <= 64U)>>
    static ctre::phoenix::StatusCode WriteBooleanArray(std::string_view name, std::array<bool, N> const &values, units::time::second_t latencySeconds = 0_s)
    {
        static_assert(sizeof(bool) == sizeof(uint8_t), "bool is not uint8_t");
        return WriteBooleanArray_Impl(name, values.data(), values.size(), latencySeconds.value());
    }
    /**
     * \brief Writes the array of booleans to the log file. The array cannot exceed 64 elements.
     * 
     * \param name Name of the signal
     * \param values Array of values to write, passed as an array of bytes
     * \param latencySeconds Latency of the signal in seconds; this value is subtracted
     *                       from the current time to get the timestamp written to the log
     * \returns Status of writing the data
     */
    template <size_t N, typename = std::enable_if_t<(N <= 64U)>>
    static ctre::phoenix::StatusCode WriteBooleanArray(std::string_view name, std::array<uint8_t, N> const &values, units::time::second_t latencySeconds = 0_s)
    {
        return WriteBooleanArray_Impl(name, values.data(), values.size(), latencySeconds.value());
    }
    /**
     * \brief Writes the array of booleans to the log file. The array cannot exceed 64 elements.
     * 
     * \param name Name of the signal
     * \param values Vector of values to write; since vector<bool> is not a boolean array,
     *               this is passed as a vector<uint8_t> instead
     * \param latencySeconds Latency of the signal in seconds; this value is subtracted
     *                       from the current time to get the timestamp written to the log
     * \returns Status of writing the data
     */
    static ctre::phoenix::StatusCode WriteBooleanArray(std::string_view name, std::vector<uint8_t> const &values, units::time::second_t latencySeconds = 0_s)
    {
        if (values.size() > 64) {
            return ctre::phoenix::StatusCode::InvalidSize;
        }
        return WriteBooleanArray_Impl(name, values.data(), values.size(), latencySeconds.value());
    }

    /**
     * \brief Writes the array of integers to the log file. The array cannot exceed 8 elements.
     * 
     * \param name Name of the signal
     * \param values Array of values to write
     * \param units Units of the signals
     * \param latencySeconds Latency of the signal in seconds; this value is subtracted
     *                       from the current time to get the timestamp written to the log
     * \returns Status of writing the data
     */
    template <size_t N, typename = std::enable_if_t<(N <= 8U)>>
    static ctre::phoenix::StatusCode WriteIntegerArray(std::string_view name, std::array<int64_t, N> const &values, std::string_view units = "", units::time::second_t latencySeconds = 0_s)
    {
        return WriteIntegerArray_Impl(name, values.data(), values.size(), units, latencySeconds.value());
    }
    /**
     * \brief Writes the array of integers to the log file. The array cannot exceed 8 elements.
     * 
     * \param name Name of the signal
     * \param values Vector of values to write
     * \param units Units of the signals
     * \param latencySeconds Latency of the signal in seconds; this value is subtracted
     *                       from the current time to get the timestamp written to the log
     * \returns Status of writing the data
     */
    static ctre::phoenix::StatusCode WriteIntegerArray(std::string_view name, std::vector<int64_t> const &values, std::string_view units = "", units::time::second_t latencySeconds = 0_s)
    {
        if (values.size() > 8) {
            return ctre::phoenix::StatusCode::InvalidSize;
        }
        return WriteIntegerArray_Impl(name, values.data(), values.size(), units, latencySeconds.value());
    }

    /**
     * \brief Writes the array of floats to the log file. The array cannot exceed 16 elements.
     * 
     * \param name Name of the signal
     * \param values Array of values to write
     * \param units Units of the signals
     * \param latencySeconds Latency of the signal in seconds; this value is subtracted
     *                       from the current time to get the timestamp written to the log
     * \returns Status of writing the data
     */
    template <size_t N, typename = std::enable_if_t<(N <= 16U)>>
    static ctre::phoenix::StatusCode WriteFloatArray(std::string_view name, std::array<float, N> const &values, std::string_view units = "", units::time::second_t latencySeconds = 0_s)
    {
        return WriteFloatArray_Impl(name, values.data(), values.size(), units, latencySeconds.value());
    }
    /**
     * \brief Writes the array of floats to the log file. The array cannot exceed 16 elements.
     * 
     * \param name Name of the signal
     * \param values Vector of values to write
     * \param units Units of the signals
     * \param latencySeconds Latency of the signal in seconds; this value is subtracted
     *                       from the current time to get the timestamp written to the log
     * \returns Status of writing the data
     */
    static ctre::phoenix::StatusCode WriteFloatArray(std::string_view name, std::vector<float> const &values, std::string_view units = "", units::time::second_t latencySeconds = 0_s)
    {
        if (values.size() > 16) {
            return ctre::phoenix::StatusCode::InvalidSize;
        }
        return WriteFloatArray_Impl(name, values.data(), values.size(), units, latencySeconds.value());
    }

    /**
     * \brief Writes the array of doubles to the log file. The array cannot exceed 8 elements.
     * 
     * \param name Name of the signal
     * \param values Array of values to write
     * \param units Units of the signals
     * \param latencySeconds Latency of the signal in seconds; this value is subtracted
     *                       from the current time to get the timestamp written to the log
     * \returns Status of writing the data
     */
    template <size_t N, typename = std::enable_if_t<(N <= 8U)>>
    static ctre::phoenix::StatusCode WriteDoubleArray(std::string_view name, std::array<double, N> const &values, std::string_view units = "", units::time::second_t latencySeconds = 0_s)
    {
        return WriteDoubleArray_Impl(name, values.data(), values.size(), units, latencySeconds.value());
    }
    /**
     * \brief Writes the array of doubles to the log file. The array cannot exceed 8 elements.
     * 
     * \param name Name of the signal
     * \param values Vector of values to write
     * \param units Units of the signals
     * \param latencySeconds Latency of the signal in seconds; this value is subtracted
     *                       from the current time to get the timestamp written to the log
     * \returns Status of writing the data
     */
    static ctre::phoenix::StatusCode WriteDoubleArray(std::string_view name, std::vector<double> const &values, std::string_view units = "", units::time::second_t latencySeconds = 0_s)
    {
        if (values.size() > 8) {
            return ctre::phoenix::StatusCode::InvalidSize;
        }
        return WriteDoubleArray_Impl(name, values.data(), values.size(), units, latencySeconds.value());
    }

private:
    static ctre::phoenix::StatusCode WriteRaw_Impl(std::string_view name, uint8_t const *data, uint8_t size, double latencySeconds);
    static ctre::phoenix::StatusCode WriteBoolean_Impl(std::string_view name, bool value, double latencySeconds);
    static ctre::phoenix::StatusCode WriteInteger_Impl(std::string_view name, int64_t value, std::string_view units, double latencySeconds);
    static ctre::phoenix::StatusCode WriteFloat_Impl(std::string_view name, float value, std::string_view units, double latencySeconds);
    static ctre::phoenix::StatusCode WriteDouble_Impl(std::string_view name, double value, std::string_view units, double latencySeconds);
    static ctre::phoenix::StatusCode WriteString_Impl(std::string_view name, std::string_view value, double latencySeconds);

    static ctre::phoenix::StatusCode WriteBooleanArray_Impl(std::string_view name, uint8_t const *values, uint8_t count, double latencySeconds);
    static ctre::phoenix::StatusCode WriteIntegerArray_Impl(std::string_view name, int64_t const *values, uint8_t count, std::string_view units, double latencySeconds);
    static ctre::phoenix::StatusCode WriteFloatArray_Impl(std::string_view name, float const *values, uint8_t count, std::string_view units, double latencySeconds);
    static ctre::phoenix::StatusCode WriteDoubleArray_Impl(std::string_view name, double const *values, uint8_t count, std::string_view units, double latencySeconds);
};

}
}
