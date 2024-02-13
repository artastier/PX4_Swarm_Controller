/**
 * @author Arthur Astier
 */
#pragma once

#include <type_traits>
#include <algorithm>

namespace PID {
    template<typename Numerical, typename= std::enable_if_t<std::is_floating_point_v<Numerical>>>
    class PID {
    public:
        explicit PID(const Numerical &max_anti_windup, const std::size_t &anti_windup_max_saturation) : max_windup(
                max_anti_windup), windup_max_saturation(anti_windup_max_saturation) {};

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

        void reset() {
            previous_error = 0.;
            integral = 0.;
        }

        void reset_derivative() {
            previous_error = 0.;
        };

        void reset_integral() {
            integral = 0.;
        }

        void setKp(const Numerical kp) {
            Kp = kp;
        }

        void setKi(const Numerical ki) {
            Ki = ki;
        }

        void setKd(const Numerical kd) {
            Kd = kd;
        }
        Numerical getKp() const {
            return Kp;
        }

        Numerical getKi() const {
            return Ki;
        }

        Numerical getKd() const {
            return Kd;
        }

        void setMaxAntiWindup(const Numerical maxAntiWindup) {
            max_windup = maxAntiWindup;
        }

    private:
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
        Numerical Kp{1.};
        Numerical Ki{0.};
        Numerical Kd{0.};
        Numerical previous_error{0.};
        Numerical integral{0.};
    private:
        Numerical max_windup{};
        std::size_t nb_saturated_integral{0u};
        std::size_t windup_max_saturation{3u};

    };
}

