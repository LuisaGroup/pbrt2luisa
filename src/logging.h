//
// Created by Mike on 2024/4/16.
//

#pragma once

#include <fmt/format.h>

namespace luisa {

template<typename... Args>
[[nodiscard]] constexpr auto format(fmt::format_string<Args...> fmt, Args &&...args) noexcept {
    return fmt::format(fmt, std::forward<Args>(args)...);
}

template<typename... Args>
void print(fmt::format_string<Args...> fmt, Args &&...args) noexcept {
    fmt::print(stdout, fmt, std::forward<Args>(args)...);
}

template<typename... Args>
void println(fmt::format_string<Args...> fmt, Args &&...args) noexcept {
    fmt::println(stdout, fmt, std::forward<Args>(args)...);
}

template<typename... Args>
void eprint(fmt::format_string<Args...> fmt, Args &&...args) noexcept {
    fmt::print(stderr, fmt, std::forward<Args>(args)...);
}

template<typename... Args>
void eprintln(fmt::format_string<Args...> fmt, Args &&...args) noexcept {
    fmt::println(stderr, fmt, std::forward<Args>(args)...);
}

template<typename... Args>
[[noreturn]] void panic(fmt::format_string<Args...> fmt, Args &&...args) noexcept {
    eprintln(fmt, std::forward<Args>(args)...);
    std::exit(EXIT_FAILURE);
}

template<typename... Args>
void expect(bool condition, fmt::format_string<Args...> fmt, Args &&...args) noexcept {
    if (!condition) { panic(fmt, std::forward<Args>(args)...); }
}

}// namespace luisa
