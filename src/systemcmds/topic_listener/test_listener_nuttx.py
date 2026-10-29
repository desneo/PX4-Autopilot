#!/usr/bin/env python3
"""编译生产非POSIX分支，以真实pipe验证NuttX listener的fd生命周期。"""

import json
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parent


class NativeListenerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="px4-listener-native-test-")
        cls.addClassCleanup(cls.temporary.cleanup)
        directory = Path(cls.temporary.name)
        source = (ROOT / "listener_main.cpp").read_text()
        production = source[source.index("void listener("):source.index("\nint listener_main(")]
        harness = r'''
#include <poll.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/resource.h>
#include <chrono>
#include <thread>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cerrno>

struct orb_metadata { const char *o_name; };
using orb_id_t = const orb_metadata *;
constexpr int PX4_OK = 0;
constexpr int ORB_MULTI_MAX_INSTANCES = 4;
constexpr float MESSAGE_TIMEOUT_S = 2.0f;
static std::string mode;
static int topic_fd = -1;
static int unsubscribed = 0;
static unsigned interval_ms = 0;
static unsigned poll_calls = 0;
static unsigned input_reads = 0;
static unsigned samples = 0;
static auto start = std::chrono::steady_clock::now();

int64_t elapsed_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();
}
#define PX4_INFO_RAW(...) fprintf(stdout, __VA_ARGS__)
#define PX4_ERR(...) do { fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } while (false)
int orb_exists(orb_id_t, unsigned i) { return i == 0 ? PX4_OK : -1; }
int orb_subscribe(orb_id_t) { return topic_fd; }
int orb_subscribe_multi(orb_id_t, unsigned) { return topic_fd; }
int orb_unsubscribe(int fd) { ++unsubscribed; return close(fd); }
int orb_set_interval(int, unsigned ms) { interval_ms = ms; return PX4_OK; }
extern "C" ssize_t __real_read(int, void *, size_t);
int listener_print_topic(orb_id_t, int fd) {
    char c = 0;
    if (__real_read(fd, &c, 1) != 1) { return -1; }
    ++samples;
    PX4_INFO_RAW("SAMPLE %u\n", samples);
    return PX4_OK;
}
extern "C" int __real_poll(struct pollfd *, nfds_t, int);
extern "C" int __wrap_poll(struct pollfd *fds, nfds_t count, int timeout) {
    ++poll_calls;
    if (mode == "poll_error") { errno = EIO; return -1; }
    if (mode == "poll_interrupted" && poll_calls == 1) { errno = EINTR; return -1; }
    if (mode == "input_error" || mode == "topic_error" || mode == "topic_hup") {
        fds[0].revents = mode == "input_error" ? POLLERR : 0;
        fds[1].revents = mode == "topic_error" ? POLLERR : mode == "topic_hup" ? POLLHUP : 0;
        return 1;
    }
    return __real_poll(fds, count, timeout);
}
extern "C" ssize_t __wrap_read(int fd, void *buffer, size_t length) {
    if (fd == STDIN_FILENO) {
        ++input_reads;
        if ((mode == "read_interrupted" || mode == "read_again" || mode == "read_error")
            && input_reads == 1) {
            errno = mode == "read_interrupted" ? EINTR : mode == "read_again" ? EAGAIN : EIO;
            return -1;
        }
    }
    return __real_read(fd, buffer, length);
}

PRODUCTION_LISTENER

int main(int argc, char **argv) {
    (void)argc;
    mode = argv[1];
    int topic[2], input[2];
    if (pipe(topic) != 0 || pipe(input) != 0) { return 2; }
    topic_fd = topic[0];
    if (dup2(input[0], STDIN_FILENO) < 0) { return 3; }
    close(input[0]);
    if (mode == "input_hup") { close(input[1]); }
    if (mode == "input_invalid") { close(STDIN_FILENO); }
    if (mode == "topic_invalid") { close(topic_fd); }
    if (mode == "input_eof") {
        const int null_fd = open("/dev/null", O_RDONLY);
        if (null_fd < 0 || dup2(null_fd, STDIN_FILENO) < 0) { return 4; }
        close(null_fd);
    }
    const bool publish = mode == "normal" || mode == "ordinary_input"
        || mode == "poll_interrupted" || mode == "read_interrupted" || mode == "read_again";
    if (mode == "ordinary_input" || mode.compare(0, 5, "read_") == 0) {
        if (write(input[1], "x", 1) != 1) { return 5; }
    } else if (mode.compare(0, 5, "quit_") == 0) {
        const char c = mode == "quit_q" ? 'q' : mode == "quit_Q" ? 'Q'
            : mode == "quit_ctrlc" ? 0x03 : 0x1b;
        if (write(input[1], &c, 1) != 1) { return 6; }
    }
    std::thread publisher;
    start = std::chrono::steady_clock::now();
    if (publish) {
        publisher = std::thread([&]() {
            for (unsigned i = 0; i < 3; ++i) {
                if (i) { std::this_thread::sleep_for(std::chrono::milliseconds(200)); }
                if (write(topic[1], "t", 1) != 1) { std::abort(); }
            }
        });
    }
    const orb_metadata metadata {"dyt_aisheng_telemetry"};
    listener(&metadata, 3, -1, 200);
    const int64_t duration_ms = elapsed_ms();
    if (publisher.joinable()) { publisher.join(); }
    struct rusage usage {};
    getrusage(RUSAGE_SELF, &usage);
    const long cpu_ms = (usage.ru_utime.tv_sec + usage.ru_stime.tv_sec) * 1000
        + (usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) / 1000;
    printf("RESULT {\"samples\":%u,\"unsubscribed\":%d,\"interval_ms\":%u,"
           "\"duration_ms\":%lld,\"cpu_ms\":%ld,\"poll_calls\":%u,\"input_reads\":%u}\n",
           samples, unsubscribed, interval_ms, static_cast<long long>(duration_ms),
           cpu_ms, poll_calls, input_reads);
}
'''.replace("PRODUCTION_LISTENER", production)
        cpp = directory / "listener_test.cpp"
        cpp.write_text(harness)
        cls.executable = directory / "listener_test"
        subprocess.run(["g++", "-std=c++14", "-O2", "-Wall", "-Wextra", "-Werror",
                        str(cpp), "-pthread", "-Wl,--wrap=poll", "-Wl,--wrap=read",
                        "-o", str(cls.executable)], check=True, capture_output=True, text=True)

    def run_listener(self, mode):
        result = subprocess.run([str(self.executable), mode], stdin=subprocess.DEVNULL,
                                capture_output=True, text=True, timeout=3.5, check=True)
        self.stdout = result.stdout
        self.stderr = result.stderr
        line = next(line for line in result.stdout.splitlines() if line.startswith("RESULT "))
        data = json.loads(line[len("RESULT "):])
        self.assertEqual(data["unsubscribed"], 1, data)
        self.assertLess(data["cpu_ms"], 250, data)
        return data

    def test_native_topic_fd_receives_three_samples(self):
        result = self.run_listener("normal")
        self.assertEqual(result["samples"], 3)
        self.assertEqual(result["interval_ms"], 200)
        self.assertGreaterEqual(result["duration_ms"], 399)
        self.assertLess(result["duration_ms"], 1200)

    def test_closed_input_pipe_and_eof_exit_without_busy_loop(self):
        for mode in ("input_hup", "input_eof"):
            with self.subTest(mode=mode):
                result = self.run_listener(mode)
                self.assertEqual(result["samples"], 0)
                self.assertLess(result["duration_ms"], 250)
                self.assertEqual(result["poll_calls"], 1)

    def test_input_and_topic_errors_exit_and_unsubscribe(self):
        for mode in ("input_invalid", "input_error", "topic_invalid", "topic_error",
                     "topic_hup", "poll_error", "read_error"):
            with self.subTest(mode=mode):
                result = self.run_listener(mode)
                self.assertEqual(result["samples"], 0)
                self.assertLess(result["duration_ms"], 250)
                self.assertEqual(result["poll_calls"], 1)
                self.assertTrue(self.stderr)

    def test_quit_characters_exit_and_unsubscribe(self):
        for mode in ("quit_q", "quit_Q", "quit_ctrlc", "quit_escape"):
            with self.subTest(mode=mode):
                result = self.run_listener(mode)
                self.assertEqual(result["samples"], 0)
                self.assertEqual(result["input_reads"], 1)
                self.assertLess(result["duration_ms"], 250)

    def test_ordinary_input_does_not_end_listener(self):
        result = self.run_listener("ordinary_input")
        self.assertEqual(result["samples"], 3)
        self.assertEqual(result["input_reads"], 1)

    def test_transient_poll_and_read_errors_keep_listening(self):
        for mode in ("poll_interrupted", "read_interrupted", "read_again"):
            with self.subTest(mode=mode):
                result = self.run_listener(mode)
                self.assertEqual(result["samples"], 3)

    def test_no_message_exits_after_timeout(self):
        result = self.run_listener("empty")
        self.assertEqual(result["samples"], 0)
        self.assertGreaterEqual(result["duration_ms"], 1999)
        self.assertLess(result["duration_ms"], 2600)
        self.assertEqual(result["poll_calls"], 1)
        self.assertIn("停止监听", self.stdout)


if __name__ == "__main__":
    unittest.main()
