#ifndef HYBRID_FG_UTILS_FMT_HPP
#define HYBRID_FG_UTILS_FMT_HPP

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>

#include <Eigen/Core>
#include <iostream>

namespace fmt
{
#ifdef __linux__
template <typename Derived>
struct formatter<Eigen::MatrixBase<Derived>>
{
    auto format(Eigen::MatrixBase<Derived>&& m, format_context& ctx) const
    {
        const std::stringstream ss{};
        ss << m;
        const std::string s = ss.str();
        return format(s, ctx);
    }
};

// template <>
// struct formatter<gtsam::Values>
// {
//     auto format(gtsam::Values&& m, format_context& ctx) -> decltype(ctx.out()) const
//     {
//         std::string s{};
//         for (auto&& [k, v] : m) {
//             s += fmt::format("{}: {}", k, v);
//         }
//         return format_to(ctx.out(), "{}", s);
//     }
// };

template <typename... T>
FMT_INLINE void println(std::FILE* f, format_string<T...> fmt, T&&... args)
{
    return fmt::print(f, "{}\n", fmt::format(fmt, std::forward<T>(args)...));
}

template <typename... T>
FMT_INLINE void println(format_string<T...> fmt, T&&... args)
{
    return fmt::println(stdout, fmt, std::forward<T>(args)...);
}
#elif __APPLE__

template <typename Derived>
struct formatter<Eigen::MatrixBase<Derived>> : ostream_formatter
{
};

#endif

}  // namespace fmt

#endif  // HYBRID_FG_UTILS_FMT_HPP
