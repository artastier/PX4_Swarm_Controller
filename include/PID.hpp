/**
 * @brief PID controller for feedback control systems.
 *
 * This PID controller implements proportional, integral, and derivative control to regulate the output of a system.
 * It is designed to handle floating-point numerical types.
 *
 * @author Arthur Astier
 */
#pragma once

#include <type_traits>
#include <algorithm>

namespace PID {
    template<typename Numerical, typename= std::enable_if_t<std::is_floating_point_v<Numerical>>>
    class PID {
    public:
        /**
         * @brief Construct a new PID controller object.
         *
         * @param max_anti_windup The maximum anti-windup value to prevent integral windup.
         * @param anti_windup_max_saturation The maximum saturation value for anti-windup reset.
         */
        explicit PID(const Numerical &max_anti_windup, const std::size_t &anti_windup_max_saturation) : max_windup(
                max_anti_windup), windup_max_saturation(anti_windup_max_saturation) {};

        /**
         * @brief Update the PID controller based on the error and time step.
         *
         * @param error The error between the desired setpoint and the current value.
         * @param dt The time step between updates.
         * @return Numerical The control command computed by the PID controller.
         */
        Numerical update(const Numerical error, const Numerical dt) {
            auto command = Kp * error;
            if (dt)
                command += Kd * (error - previous_error) / dt;
            // Trapezoid method
            integral += dt * (previous_error + error) / 2;
            anti_windup();
            command += Ki * integral;
            return command;
        };

        /**
         * @brief Reset the PID controller.
         */
        void reset() {
            previous_error = 0.;
            integral = 0.;
        }

        /**
         * @brief Reset the derivative term of the PID controller.
         */
        void reset_derivative() {
            previous_error = 0.;
        };

        /**
         * @brief Reset the integral term of the PID controller.
         */
        void reset_integral() {
            integral = 0.;
        }

        /**
         * @brief Set the proportional gain of the PID controller.
         *
         * @param kp The proportional gain value.
         */
        void setKp(const Numerical kp) {
            Kp = kp;
        }

        /**
         * @brief Set the integral gain of the PID controller.
         *
         * @param ki The integral gain value.
         */
        void setKi(const Numerical ki) {
            Ki = ki;
        }

        /**
         * @brief Set the derivative gain of the PID controller.
         *
         * @param kd The derivative gain value.
         */
        void setKd(const Numerical kd) {
            Kd = kd;
        }

        /**
         * @brief Get the proportional gain of the PID controller.
         *
         * @return Numerical The proportional gain value.
         */
        Numerical getKp() const {
            return Kp;
        }

        /**
         * @brief Get the integral gain of the PID controller.
         *
         * @return Numerical The integral gain value.
         */
        Numerical getKi() const {
            return Ki;
        }

        /**
         * @brief Get the derivative gain of the PID controller.
         *
         * @return Numerical The derivative gain value.
         */
        Numerical getKd() const {
            return Kd;
        }

        /**
         * @brief Set the maximum anti-windup value.
         *
         * @param maxAntiWindup The maximum anti-windup value.
         */
        void setMaxAntiWindup(const Numerical maxAntiWindup) {
            max_windup = maxAntiWindup;
        }

    private:
        /**
         * @brief Apply anti-windup mechanism to prevent integral windup.
         */
        void anti_windup() {
            const auto saturation{std::max(std::min(integral, max_windup), -max_windup)};
            if (std::abs(saturation) == max_windup) {
                ++nb_saturated_integral;
                if (nb_saturated_integral > windup_max_saturation) {
                    nb_saturated_integral = 0u;
                    reset_integral();
                }
            }else{
                integral = saturation;
            }
        }

    private:
        Numerical Kp{1.}; /**< Proportional gain */
        Numerical Ki{0.}; /**< Integral gain */
        Numerical Kd{0.}; /**< Derivative gain */
        Numerical previous_error{0.}; /**< Previous error */
        Numerical integral{0.}; /**< Integral of error */
    private:
        Numerical max_windup{}; /**< Maximum anti-windup value */
        std::size_t nb_saturated_integral{0u}; /**< Number of times integral term is saturated */
        std::size_t windup_max_saturation{3u}; /**< Maximum number of times integral term is saturated before reset */
    };
}
