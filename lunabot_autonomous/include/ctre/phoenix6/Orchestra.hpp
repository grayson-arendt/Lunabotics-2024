/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */

#include "ctre/phoenix6/hardware/ParentDevice.hpp"

namespace ctre {
namespace phoenix6 {

/**
 * \brief Orchestra is used to play music through devices. It uses a "Chirp" (.chrp)
 * music file that can be generated using Phoenix Tuner. Chirp files are generated
 * from standard MIDI files.
 *
 * Any Chirp file located in the src/main/deploy directory of your FRC project will
 * automatically be copied to the roboRIO on code deploy.
 *
 * The robot must be enabled to play music. Additionally, devices playing in Orchestra
 * will not run any other control requests while Orchestra is running. Users can #Pause
 * or #Stop the Orchestra to re-enable device control.
 *
 * \details Each device can only play a single track within the music file. For multi-track
 * files, multiple devices are needed. Devices can be added with an explicit track
 * number. Otherwise, the first track will be played through the first Talon FX added,
 * the second track will be played through the second Talon FX added, etc.
 *
 * To use Orchestra:
 * - Add the Talon FXs to be used as instruments using #AddInstrument.
 * - Load the Chirp file to be played using #LoadMusic.
 * Both of these can also be done in the Orchestra constructor.
 *
 * Once ready, the Orchestra can be controlled using #Play/#Pause/#Stop. New music
 * files can be loaded at any time.
 */
class Orchestra {
private:
    uint16_t _id{};

public:
    /**
     * \brief Constructor for a new Orchestra.
     */
    Orchestra();
    /**
     * \brief Constructor for a new Orchestra using the given Chirp file.
     *
     * \param filepath The path to the music file to immediately load into the orchestra.
     */
    Orchestra(char const *filepath) : Orchestra{}
    {
        LoadMusic(filepath);
    }
    /**
     * \brief Constructor for a new Orchestra using the given Chirp file.
     *
     * \param instruments A vector of device addresses that will be used as instruments in the orchestra.
     */
    Orchestra(std::vector<hardware::ParentDevice *> const &instruments) : Orchestra{}
    {
        for (auto instrument : instruments) {
            AddInstrument(*instrument);
        }
    }
    /**
     * \brief Constructor for a new Orchestra using the given Chirp file.
     *
     * \param instruments An array of device addresses that will be used as instruments in the orchestra.
     */
    template <size_t N>
    Orchestra(std::array<hardware::ParentDevice *, N> const &instruments) : Orchestra{}
    {
        for (auto instrument : instruments) {
            AddInstrument(*instrument);
        }
    }
    /**
     * \brief Constructor for a new Orchestra using the given Chirp file.
     *
     * \param instruments A vector of device addresses that will be used as instruments in the orchestra.
     * \param filepath The path to the music file to immediately load into the orchestra.
     */
    Orchestra(std::vector<hardware::ParentDevice *> const &instruments, char const *filepath) : Orchestra{}
    {
        for (auto instrument : instruments) {
            AddInstrument(*instrument);
        }
        LoadMusic(filepath);
    }
    /**
     * \brief Constructor for a new Orchestra using the given Chirp file.
     *
     * \param instruments An array of device addresses that will be used as instruments in the orchestra.
     * \param filepath The path to the music file to immediately load into the orchestra.
     */
    template <size_t N>
    Orchestra(std::array<hardware::ParentDevice *, N> const &instruments, char const *filepath) : Orchestra{}
    {
        for (auto instrument : instruments) {
            AddInstrument(*instrument);
        }
        LoadMusic(filepath);
    }

    ~Orchestra();

    /**
     * \brief Adds an instrument to the orchestra.
     *
     * \param instrument The device to add to the orchestra
     * \returns Status code of adding the device
     */
    ctre::phoenix::StatusCode AddInstrument(hardware::ParentDevice const &instrument);

    /**
     * \brief Adds an instrument to the orchestra on the given track.
     *
     * \param instrument The device to add to the orchestra
     * \param trackNumber The track number the device should play, starting at 0
     * \returns Status code of adding the device
     */
    ctre::phoenix::StatusCode AddInstrument(hardware::ParentDevice const &instrument, uint16_t trackNumber);

    /**
     * \brief Clears all instruments in the orchestra.
     *
     * \returns Status code of clearing all devices
     */
    ctre::phoenix::StatusCode ClearInstruments();

    /**
     * \brief Loads a Chirp file at the specified file path.
     *
     * If the Chirp file is inside your "src/main/deploy" directory, it will be
     * automatically deployed to a default directory on the roboRIO when you
     * deploy code. For these files, the name and file extension is sufficient.
     *
     * A Chirp file can be created from a MIDI file using Phoenix Tuner.
     *
     * \param filepath The path to the Chirp file
     * \returns Status code of loading the Chirp file
     */
    ctre::phoenix::StatusCode LoadMusic(char const *filepath);

    /**
     * \brief Plays the loaded music file. If the player is paused, this will resume
     * the orchestra.
     *
     * \returns Status code of playing the orchestra
     */
    ctre::phoenix::StatusCode Play();

    /**
     * \brief Pauses the loaded music file. This saves the current position in the
     * track so it can be resumed later.
     *
     * \returns Status code of pausing the orchestra
     */
    ctre::phoenix::StatusCode Pause();

    /**
     * \brief Stops the loaded music file. This resets the current position in the
     * track to the start.
     *
     * \returns Status code of stopping the orchestra
     */
    ctre::phoenix::StatusCode Stop();

    /**
     * \brief Gets whether the current track is actively playing.
     *
     * \returns true if Orchestra is playing the music file
     */
    bool IsPlaying() const;

    /**
     * \brief Gets the current timestamp of the music file. The timestamp will reset
     * to zero whenever #LoadMusic or #Stop is called.
     *
     * \details If #IsPlaying returns false, this method can be used to determine
     * if the music is stopped or paused.
     *
     * \returns The current timestamp of the music file, in seconds
     */
    double GetCurrentTime() const;
};

} // namespace phoenix6
} // namespace ctre
