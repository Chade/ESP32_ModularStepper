#ifndef STEPPER_STOP_CONTROLLER_HPP
#define STEPPER_STOP_CONTROLLER_HPP

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

namespace Stepper {
namespace Generic {

class IStoppable {
public:
    virtual ~IStoppable() = default;

    virtual bool pause() = 0;
    virtual bool resume() = 0;
    virtual void fastStop() = 0;
    virtual void fastStop(float deceleration) = 0;
    virtual void emergencyStop(bool disableDriver = false) = 0;
    virtual bool clearEmergencyStop(bool enableDriver = false) = 0;
};

class StopController {
public:
    static constexpr int16_t ALL_TARGETS = -1;

    StopController() = default;
    ~StopController() = default;

    StopController(const StopController&) = delete;
    StopController& operator=(const StopController&) = delete;

    // Registry management (valid IDs are >= 0; negative IDs and duplicates are rejected)
    bool registerTarget(int16_t id, IStoppable& target);
    bool unregisterTarget(int16_t id);
    bool unregisterTarget(const IStoppable& target);
    bool hasTarget(int16_t id) const;
    size_t targetCount() const;

    // Single-target operations
    bool pause(int16_t id);
    bool resume(int16_t id);
    bool fastStop(int16_t id, float deceleration = 0.0f);
    bool emergencyStop(int16_t id, bool disableDriver = false);
    bool clearEmergencyStop(int16_t id, bool enableDriver = false);

    // Broadcast operations across all registered targets
    void pauseAll();
    void resumeAll();
    void fastStopAll(float deceleration = 0.0f);
    void emergencyStopAll(bool disableDrivers = false);
    void clearEmergencyStopAll(bool enableDrivers = false);

private:
    struct Entry {
        int16_t id;
        IStoppable* target;
    };

    IStoppable* findTargetLocked(int16_t id) const;

    mutable std::mutex mutex_;
    std::vector<Entry> targets_;
};

} // namespace Generic
} // namespace Stepper

#endif // STEPPER_STOP_CONTROLLER_HPP
