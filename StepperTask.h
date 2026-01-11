#ifndef STEPPER_TASK_H
#define STEPPER_TASK_H

#include <algorithm>
#include <cfloat>
#include <climits>
#include <cmath>
#include <cstdint>
#include <deque>

namespace Stepper
{

    enum class State : uint8_t
    {
        UNDEFINED = 0,
        MOVING = 1,
        ACCELERATING = 3,
        DECELERATING = 5,
        PAUSED = 16,
        STOPPED = 32,
        INHIBITED = 96,
        EMERGENCYSTOP = 224
    };

    enum class TaskState : uint8_t
    {
        UNDEFINED = 0,
        ACTIVE = 1,
        PAUSED = 2,
        STOPPED = 3,
    };

    struct Task
    {
        /* Define a task for the step generator to execute
         * There are different tasks possible:
         * 1) ticks > 0
         *    Execute a movement for the given number of ticks
         *    1.1) ticks_per_second > 0
         *         1.2.1) acceleration != 0 && deceleration == 0
         *                Accelerate until ticks_per_seconds is reached and until given number of ticks have passed
         *                If there is no consecutive task, motor will come to an abrupt stop (as there is no deceleration specified)
         *         1.2.2) acceleration != 0 && deceleration != 0
         *                Accelerate and until ticks_per_second is reached then decelerate to standstill until given number of ticks have passed
         *                If current ticks_per_second is higher then requested ticks_per_second, the motor will first decelerate to ticks_per_second and keep it before decelerating again.
         *                If current ticks_per_second is too high to come to standstill with given deceleration, the motor will come to an abrupt stop, if there is no consecutive task
         *         1.2.3) acceleration == 0 && deceleration != 0
         *                Decelerate to standstill until given number of ticks have passed
         *                If motor is in standstill, it will abruptly accelerate to ticks_per_second
         *                If current velocity is too high to come to standstill with the given deceleration, the motor will come to an abrupt stop, if there is no consecutive task
         *    1.2) ticks_per_Second == 0
         *         1.2.1) acceleration != 0 && deceleration == 0
         *                Accelerate until given number of ticks have passed
         *                If there is no consecutive task, motor will come to an abrupt stop (as there is no deceleration specified)
         *         1.2.2) acceleration != 0 && deceleration != 0
         *                Accelerate and then decelerate to standstill until given number of ticks have passed
         *                The maximum motor speed could be surpassed
         *         1.2.3) acceleration == 0 && deceleration != 0
         *                Decelerate to standstill until given number of ticks have passed
         *                If motor is already in standstill, no movement will be executed
         *                If current velocity is too high to come to standstill with the given deceleration, the motor will come to an abrupt stop, if there is no consecutive task
         *
         */

    public:
        uint64_t id{0};
        uint64_t steps{0};
        uint16_t steps_per_second{0};
        uint16_t acceleration{0};
        uint16_t deceleration{0};
        Direction direction{Direction::NEUTRAL};
    };

    class CoreTask : public Task
    {
    public:
        CoreTask() = default;

        CoreTask(const Task &task) : Task(task)
        {
            state = TaskState::UNDEFINED;
        };

        CoreTask &operator=(const Task &task)
        {
            if (this != &task)
            {
                Task::operator=(task);
                state = TaskState::UNDEFINED;
            }

            return *this;
        };

        TaskState getState()
        {
            return state;
        };

        void setState(TaskState newState)
        {
            state = newState;
        };

    private:
        TaskState state;
    };

    class TaskQueue
    {
    public:
        TaskQueue() = default;
        ~TaskQueue() = default;

        void enqueueTask(const Task &new_task)
        {
            m_taskQueue.push_back(new_task);
        };

        Task dequeueTask()
        {
            Task task = m_taskQueue.front();
            m_taskQueue.pop_front();
            return task;
        };

        bool updateTask(uint64_t task_id, const Task &new_task)
        {
            auto matchId = [task_id](const auto &task)
            { return task.id == task_id; };
            auto it = std::find_if(m_taskQueue.begin(), m_taskQueue.end(), matchId);

            if (it != m_taskQueue.end())
            {
                *it = new_task;
                return true;
            }
            return false;
        };

        bool deleteTask(uint64_t task_id)
        {
            auto matchId = [task_id](const auto &task)
            { return task.id == task_id; };
            auto it = std::remove_if(m_taskQueue.begin(), m_taskQueue.end(), matchId);

            if (it != m_taskQueue.end())
            {
                m_taskQueue.erase(it, m_taskQueue.end());
                return true;
            }
            return false;
        };

        Task &front()
        {
            return m_taskQueue.front();
        };

        Task &back()
        {
            return m_taskQueue.back();
        };

        void clear()
        {
            m_taskQueue.clear();
        };

        size_t size()
        {
            return m_taskQueue.size();
        };

        bool isEmpty()
        {
            return m_taskQueue.empty();
        };

    private:
        std::deque<Task> m_taskQueue;
    };
}

#endif // STEPPER_TASK_H