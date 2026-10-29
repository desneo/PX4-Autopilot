#!/usr/bin/env python3
"""编译生产listener，隔离验证POSIX输入、uORB间隔、墙钟超时与退订。"""

import json
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parent


class PosixListenerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="px4-listener-test-")
        cls.addClassCleanup(cls.temporary.cleanup)
        directory = Path(cls.temporary.name)
        source = (ROOT / "listener_main.cpp").read_text()
        production = source[source.index("void listener("):source.index("\nint listener_main(")]
        harness = r'''
#include <poll.h>
#include <unistd.h>
#include <sys/resource.h>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cerrno>
#include <ctime>

struct orb_metadata { const char *o_name; };
using orb_id_t = const orb_metadata *;
constexpr int PX4_OK = 0;
constexpr int ORB_MULTI_MAX_INSTANCES = 4;
constexpr float MESSAGE_TIMEOUT_S = 2.0f;
static std::string mode;
static int input_fd = STDIN_FILENO;
static FILE *output = stdout;
static int unsubscribed = 0;
static unsigned interval_ms = 0;
static unsigned poll_calls = 0;
static unsigned subscription_poll_calls = 0;
static std::vector<int64_t> samples;
static auto start = std::chrono::steady_clock::now();
static int64_t last_copy_ms = -100000;

int64_t elapsed_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();
}
int get_stdin() { return input_fd; }
FILE *get_stdout(bool *) { return output; }
#define PX4_INFO_RAW(...) fprintf(get_stdout(nullptr), __VA_ARGS__)
#define PX4_ERR(...) do { fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } while (false)
int orb_exists(orb_id_t, unsigned i) { return i == 0 ? PX4_OK : -1; }
// 该句柄故意不是操作系统fd，原生poll会立即产生POLLNVAL。
int orb_subscribe(orb_id_t) { return 511; }
int orb_subscribe_multi(orb_id_t, unsigned) { return mode == "subscribe_error" ? -1 : 511; }
int orb_unsubscribe(int) { ++unsubscribed; return PX4_OK; }
int orb_set_interval(int, unsigned ms) {
    interval_ms = ms;
    return mode == "interval_error" ? -1 : PX4_OK;
}
int orb_check(int, bool *updated) {
    if (mode == "check_error") { return -1; }
    // 模拟1 kHz发布及uORB订阅间隔；消息时钟不参与listener超时。
    *updated = mode != "empty" && mode != "local_eof_timeout"
        && elapsed_ms() - last_copy_ms >= interval_ms && elapsed_ms() > last_copy_ms;
    return PX4_OK;
}
int listener_print_topic(orb_id_t, int) {
    last_copy_ms = elapsed_ms();
    samples.push_back(last_copy_ms);
    PX4_INFO_RAW("SAMPLE %lld\n", static_cast<long long>(last_copy_ms));
    return PX4_OK;
}
extern "C" int __real_poll(struct pollfd *, nfds_t, int);
extern "C" int __wrap_poll(struct pollfd *fds, nfds_t count, int timeout) {
    ++poll_calls;
    for (nfds_t i = 0; i < count; ++i) {
        if (fds[i].fd == 511) { ++subscription_poll_calls; }
    }
    if (mode == "poll_error") { errno = EIO; return -1; }
    if (mode == "interrupted" && poll_calls == 1) { errno = EINTR; return -1; }
    return __real_poll(fds, count, timeout);
}

PRODUCTION_LISTENER

int main(int argc, char **argv) {
    mode = argv[1];
    close(511);
    close(510);
    int remote_pipe[2] {-1, -1};
    std::thread input_thread;
    if (mode.compare(0, 7, "remote_") == 0 || mode == "empty") {
        if (pipe(remote_pipe) != 0) { return 2; }
        input_fd = remote_pipe[0];
        if (mode == "remote_output") {
            output = tmpfile();
            if (!output) { return 3; }
        } else if (mode != "empty") {
            input_thread = std::thread([&]() {
                std::this_thread::sleep_for(std::chrono::milliseconds(80));
                if (mode == "remote_closed") { close(remote_pipe[1]); return; }
                const char c = mode == "remote_q" ? 'q' : mode == "remote_Q" ? 'Q'
                    : mode == "remote_ctrlc" ? 0x03 : 0x1b;
                if (write(remote_pipe[1], &c, 1) != 1) { std::abort(); }
            });
        }
    } else if (mode == "input_invalid") {
        input_fd = 510;
    }
    const orb_metadata metadata {"dyt_aisheng_telemetry"};
    start = std::chrono::steady_clock::now();
    listener(&metadata, argc > 2 ? static_cast<unsigned>(atoi(argv[2])) : 3,
             mode == "single_explicit_instance" ? 0 : -1,
             argc > 3 ? static_cast<unsigned>(atoi(argv[3])) : 200);
    const int64_t duration_ms = elapsed_ms();
    if (input_thread.joinable()) { input_thread.join(); }
    if (output != stdout) {
        fflush(output);
        rewind(output);
        puts("CAPTURE_BEGIN");
        char buffer[256];
        size_t length;
        while ((length = fread(buffer, 1, sizeof(buffer), output)) > 0) {
            fwrite(buffer, 1, length, stdout);
        }
        puts("CAPTURE_END");
        fclose(output);
    }
    struct rusage usage {};
    getrusage(RUSAGE_SELF, &usage);
    const long cpu_ms = (usage.ru_utime.tv_sec + usage.ru_stime.tv_sec) * 1000
        + (usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) / 1000;
    printf("RESULT {\"samples\":%zu,\"unsubscribed\":%d,\"interval_ms\":%u,"
           "\"duration_ms\":%lld,\"cpu_ms\":%ld,\"poll_calls\":%u,"
           "\"subscription_poll_calls\":%u,\"sample_times\":[",
           samples.size(), unsubscribed, interval_ms, static_cast<long long>(duration_ms),
           cpu_ms, poll_calls, subscription_poll_calls);
    for (size_t i = 0; i < samples.size(); ++i) {
        printf("%s%lld", i ? "," : "", static_cast<long long>(samples[i]));
    }
    puts("]}");
}
'''.replace("PRODUCTION_LISTENER", production)
        cpp = directory / "listener_test.cpp"
        cpp.write_text(harness)
        cls.executable = directory / "listener_test"
        subprocess.run(["g++", "-std=c++14", "-O2", "-Wall", "-Wextra", "-Werror",
                        "-D__PX4_POSIX", str(cpp), "-pthread", "-Wl,--wrap=poll",
                        "-o", str(cls.executable)], check=True, capture_output=True, text=True)

    def run_listener(self, mode, count=3, interval_ms=200):
        result = subprocess.run([str(self.executable), mode, str(count), str(interval_ms)], stdin=subprocess.DEVNULL,
                                capture_output=True, text=True, timeout=3.5, check=True)
        self.stdout = result.stdout
        self.stderr = result.stderr
        line = next(line for line in result.stdout.splitlines() if line.startswith("RESULT "))
        data = json.loads(line.removeprefix("RESULT "))
        self.assertEqual(data["subscription_poll_calls"], 0, data)
        self.assertLess(data["cpu_ms"], 250, data)
        return data

    def test_unopened_cdev_handle_receives_three_samples_at_five_hz(self):
        result = self.run_listener("samples")
        self.assertEqual(result["samples"], 3)
        self.assertEqual(result["unsubscribed"], 1)
        self.assertEqual(result["interval_ms"], 200)
        self.assertGreaterEqual(result["duration_ms"], 400)
        self.assertLess(result["duration_ms"], 1200)
        self.assertTrue(all(b - a >= 200 for a, b in zip(result["sample_times"],
                                                       result["sample_times"][1:])))

    def test_ten_millisecond_subscription_is_not_limited_to_fifty_hz(self):
        result = self.run_listener("samples", count=31, interval_ms=10)
        self.assertEqual(result["samples"], 31)
        self.assertEqual(result["interval_ms"], 10)
        self.assertEqual(result["unsubscribed"], 1)
        self.assertGreaterEqual(result["duration_ms"], 300)
        # 允许调度延迟；固定20 ms等待至少需要600 ms。
        self.assertLess(result["duration_ms"], 550)

    def test_one_millisecond_subscription_keeps_fast_updates(self):
        result = self.run_listener("samples", count=101, interval_ms=1)
        self.assertEqual(result["samples"], 101)
        self.assertEqual(result["interval_ms"], 1)
        self.assertEqual(result["unsubscribed"], 1)
        self.assertGreaterEqual(result["duration_ms"], 100)
        self.assertLess(result["duration_ms"], 600)

    def test_unlimited_subscription_uses_short_wait_between_publications(self):
        result = self.run_listener("samples", count=101, interval_ms=0)
        self.assertEqual(result["samples"], 101)
        self.assertEqual(result["interval_ms"], 0)
        self.assertEqual(result["unsubscribed"], 1)
        self.assertGreaterEqual(result["duration_ms"], 100)
        self.assertLess(result["duration_ms"], 600)

    def test_local_eof_waits_for_wall_clock_timeout_without_busy_loop(self):
        result = self.run_listener("local_eof_timeout")
        self.assertEqual(result["samples"], 0)
        self.assertEqual(result["unsubscribed"], 1)
        # 生产墙钟与测试计时分别取整到毫秒，边界允许1 ms量化差。
        self.assertGreaterEqual(result["duration_ms"], 1999)
        self.assertLess(result["duration_ms"], 2600)
        self.assertLess(result["poll_calls"], 150)
        self.assertIn("停止监听", self.stdout)

    def test_open_input_without_messages_exits_on_wall_clock_timeout(self):
        result = self.run_listener("empty")
        self.assertEqual(result["samples"], 0)
        self.assertEqual(result["unsubscribed"], 1)
        # 生产墙钟与测试计时分别取整到毫秒，边界允许1 ms量化差。
        self.assertGreaterEqual(result["duration_ms"], 1999)
        self.assertLess(result["duration_ms"], 2600)
        self.assertLess(result["poll_calls"], 150)

    def test_remote_quit_characters_and_disconnect_unsubscribe(self):
        for mode in ("remote_q", "remote_Q", "remote_ctrlc", "remote_escape", "remote_closed"):
            with self.subTest(mode=mode):
                result = self.run_listener(mode, 1000000)
                self.assertEqual(result["samples"], 1)
                self.assertEqual(result["unsubscribed"], 1)
                self.assertLess(result["duration_ms"], 500)

    def test_remote_output_includes_clear_screen_in_command_stream(self):
        result = self.run_listener("remote_output")
        self.assertEqual(result["samples"], 3)
        captured = self.stdout.split("CAPTURE_BEGIN\n", 1)[1].split("CAPTURE_END", 1)[0]
        self.assertEqual(captured.count("\x1b[2J"), 3)
        self.assertEqual(captured.count("TOPIC: dyt_aisheng_telemetry"), 3)

    def test_poll_and_subscription_errors_exit_and_unsubscribe(self):
        for mode in ("input_invalid", "poll_error", "check_error", "interval_error"):
            with self.subTest(mode=mode):
                result = self.run_listener(mode)
                self.assertEqual(result["unsubscribed"], 1)
                self.assertEqual(result["samples"], 0)
                self.assertLess(result["duration_ms"], 250)
                self.assertTrue(self.stderr)

    def test_failed_subscription_does_not_unsubscribe_invalid_handle(self):
        result = self.run_listener("subscribe_error")
        self.assertEqual(result["unsubscribed"], 0)
        self.assertEqual(result["samples"], 0)

    def test_interrupted_poll_continues_to_receive_samples(self):
        result = self.run_listener("interrupted")
        self.assertEqual(result["samples"], 3)
        self.assertEqual(result["unsubscribed"], 1)

    def test_explicit_single_topic_instance_uses_valid_update_path(self):
        result = self.run_listener("single_explicit_instance", 1)
        self.assertEqual(result["samples"], 1)
        self.assertEqual(result["unsubscribed"], 1)

    def test_default_single_sample_keeps_snapshot_behavior(self):
        result = self.run_listener("samples", 1)
        self.assertEqual(result["samples"], 1)
        self.assertEqual(result["unsubscribed"], 1)
        self.assertEqual(result["poll_calls"], 0)


if __name__ == "__main__":
    unittest.main()
