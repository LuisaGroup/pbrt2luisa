//
// Created by Mike on 2024/4/16.
//

#pragma once

#include <print>

namespace luisa {

template<typename... Args>
void print(const std::format_string<Args...> fmt, Args &&...args) noexcept {
    std::print(stdout, fmt, std::forward<Args>(args)...);
}

template<typename... Args>
void println(const std::format_string<Args...> fmt, Args &&...args) noexcept {
    std::println(stdout, fmt, std::forward<Args>(args)...);
}

template<typename... Args>
void eprint(const std::format_string<Args...> fmt, Args &&...args) noexcept {
    std::print(stderr, fmt, std::forward<Args>(args)...);
}

template<typename... Args>
void eprintln(const std::format_string<Args...> fmt, Args &&...args) noexcept {
    std::println(stderr, fmt, std::forward<Args>(args)...);
}

template<typename... Args>
[[noreturn]] void panic(const std::format_string<Args...> fmt, Args &&...args) noexcept {
    eprintln(fmt, std::forward<Args>(args)...);
    std::exit(EXIT_FAILURE);
}

template<typename... Args>
void expect(bool condition, const std::format_string<Args...> fmt, Args &&...args) noexcept {
    if (!condition) { panic(fmt, std::forward<Args>(args)...); }
}

}// namespace luisa
