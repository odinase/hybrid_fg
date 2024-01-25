#ifndef HYBRID_FG_UTILS_FMT_HPP
#define HYBRID_FG_UTILS_FMT_HPP

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>

#include <Eigen/Core>
#include <iostream>

namespace fmt
{
template <typename Derived>
struct formatter<Eigen::MatrixBase<Derived>>
{
    auto format(Eigen::MatrixBase<Derived>&& m, format_context& ctx) const
    {
        std::stringstream ss{};
        ss << m;
        const std::string s = ss.str();
        return format(s, ctx);
    }
};

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
}  // namespace fmt

#endif  // HYBRID_FG_UTILS_FMT_HPP