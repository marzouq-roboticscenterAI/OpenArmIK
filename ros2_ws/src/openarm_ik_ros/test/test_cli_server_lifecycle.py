#!/usr/bin/env python3
"""Verify bounded CLI behavior across action-server and context loss."""

import argparse
import os
import signal
import subprocess
import sys
import tempfile
import time


def write_marker(name):
    marker = os.environ['OPENARM_CLI_TEST_MARKER']
    with open(marker, 'a', encoding='utf-8') as stream:
        stream.write(name + '\n')


def run_server(mode):
    import rclpy
    from openarm_control_msgs.action import MoveJoint
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import ExternalShutdownException
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    class Fixture(Node):

        def __init__(self):
            super().__init__('openarm_cli_lifecycle_fixture')
            self.action_server = ActionServer(
                self,
                MoveJoint,
                '/openarm_ik/move_joint',
                execute_callback=self.execute,
                goal_callback=lambda _: GoalResponse.ACCEPT,
                cancel_callback=self.cancel,
                handle_accepted_callback=self.accepted,
                callback_group=ReentrantCallbackGroup(),
            )
            write_marker('ready')

        def accepted(self, goal_handle):
            write_marker('accepted')
            if mode != 'queued':
                goal_handle.execute()

        def cancel(self, _goal_handle):
            write_marker('cancel_requested')
            return CancelResponse.ACCEPT

        def execute(self, goal_handle):
            write_marker('started')
            if mode == 'slow':
                deadline = time.monotonic() + 1.0
                while rclpy.ok() and time.monotonic() < deadline:
                    time.sleep(0.02)
                result = MoveJoint.Result()
                if rclpy.ok():
                    goal_handle.succeed()
                    result.outcome = MoveJoint.Result.OUTCOME_COMPLETED
                    result.command_id = 41
                    result.reason = 'fixture completed'
                return result
            if mode == 'settling':
                feedback = MoveJoint.Feedback()
                feedback.measured_progress = 0.99
                goal_handle.publish_feedback(feedback)
                write_marker('settling')
            while rclpy.ok():
                if mode == 'cancel_race' and goal_handle.is_cancel_requested:
                    time.sleep(0.05)
                    goal_handle.canceled()
                    result = MoveJoint.Result()
                    result.outcome = MoveJoint.Result.OUTCOME_CANCELED
                    result.reason = 'fixture canceled'
                    write_marker('canceled')
                    return result
                time.sleep(0.02)
            return MoveJoint.Result()

    rclpy.init()
    fixture = Fixture()
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        executor.add_node(fixture)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown(timeout_sec=1.0)
        fixture.action_server.destroy()
        fixture.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def wait_marker(path, expected, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with open(path, encoding='utf-8') as stream:
                if expected in stream.read().splitlines():
                    return
        except FileNotFoundError:
            pass
        time.sleep(0.02)
    raise AssertionError(f'timed out waiting for fixture marker {expected}')


def stop(process):
    if process.poll() is not None:
        return process.communicate(timeout=0.1)
    process.send_signal(signal.SIGINT)
    try:
        return process.communicate(timeout=2.0)
    except subprocess.TimeoutExpired:
        process.kill()
        return process.communicate(timeout=2.0)


def start_case(executable, mode, domain, directory, context_shutdown=False):
    marker = os.path.join(directory, f'{mode}.marker')
    environment = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = str(domain)
    environment['OPENARM_CLI_TEST_MARKER'] = marker
    if context_shutdown:
        environment['OPENARM_CLI_TEST_CONTEXT_SHUTDOWN'] = '1'
    server = subprocess.Popen(
        [sys.executable, __file__, '--server', mode],
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    try:
        wait_marker(marker, 'ready')
        client = subprocess.Popen(
            [executable, 'move-joint', 'openarm_left_joint1', '0.1'],
            env=environment,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        return server, client, marker
    except (AssertionError, OSError, subprocess.SubprocessError):
        stop(server)
        raise


def verify_server_loss(executable, mode, domain, directory):
    server, client, marker = start_case(executable, mode, domain, directory)
    phases = {
        'queued': 'accepted',
        'started': 'started',
        'settling': 'settling',
    }
    phase = phases[mode]
    try:
        wait_marker(marker, phase)
        started = time.monotonic()
        stop(server)
        stdout, stderr = client.communicate(timeout=2.0)
        elapsed = time.monotonic() - started
        assert client.returncode == 8, (
            mode, client.returncode, stdout, stderr)
        assert 'action server lost after goal acceptance' in stderr, (
            mode, stderr)
        assert 'terminal result timeout' not in stderr, (mode, stderr)
        assert elapsed <= 2.0, (mode, elapsed)
        assert client.poll() is not None
    finally:
        stop(server)
        if client.poll() is None:
            client.kill()
            client.communicate(timeout=2.0)


def verify_slow_success(executable, domain, directory):
    server, client, marker = start_case(executable, 'slow', domain, directory)
    try:
        wait_marker(marker, 'started')
        stdout, stderr = client.communicate(timeout=3.0)
        assert client.returncode == 0, (client.returncode, stdout, stderr)
        assert 'completed command_id=41' in stdout
        assert 'lost' not in stderr
    finally:
        stop(server)
        if client.poll() is None:
            client.kill()
            client.communicate(timeout=2.0)


def verify_cancel_race(executable, domain, directory):
    server, client, marker = start_case(
        executable, 'cancel_race', domain, directory)
    try:
        wait_marker(marker, 'started')
        stdout, stderr = client.communicate(timeout=4.0)
        assert client.returncode == 6, (client.returncode, stdout, stderr)
        assert 'fixture canceled' in stderr
        wait_marker(marker, 'cancel_requested')
        wait_marker(marker, 'canceled')
        assert 'lost' not in stderr
    finally:
        stop(server)
        if client.poll() is None:
            client.kill()
            client.communicate(timeout=2.0)


def verify_context_shutdown(executable, domain, directory):
    server, client, marker = start_case(
        executable, 'started', domain, directory, context_shutdown=True)
    try:
        wait_marker(marker, 'started')
        started = time.monotonic()
        stdout, stderr = client.communicate(timeout=2.0)
        elapsed = time.monotonic() - started
        assert client.returncode == 8, (client.returncode, stdout, stderr)
        assert 'ROS context shut down after goal acceptance' in stderr
        assert 'terminal result timeout' not in stderr
        assert elapsed <= 2.0, elapsed
    finally:
        stop(server)
        if client.poll() is None:
            client.kill()
            client.communicate(timeout=2.0)


def run_test(executable, production_executable):
    # Keep fixture discovery disjoint from the 100-199 and 180-229 ranges used
    # by the existing ROS contract and SIGINT tests.
    base_domain = 10 + os.getpid() % 5
    with tempfile.TemporaryDirectory(
            prefix='openarm-cli-lifecycle-') as directory:
        for offset, mode in enumerate(('queued', 'started', 'settling')):
            verify_server_loss(
                production_executable, mode, base_domain + offset, directory)
        verify_slow_success(executable, base_domain + 3, directory)
        verify_cancel_race(executable, base_domain + 4, directory)
        verify_context_shutdown(executable, base_domain + 5, directory)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--executable')
    parser.add_argument('--production-executable')
    parser.add_argument('--server', choices=(
        'queued', 'started', 'settling', 'slow', 'cancel_race'))
    arguments = parser.parse_args()
    if arguments.server:
        run_server(arguments.server)
    else:
        if not arguments.executable:
            parser.error('--executable is required for the test driver')
        if not arguments.production_executable:
            parser.error(
                '--production-executable is required for the test driver')
        run_test(arguments.executable, arguments.production_executable)


if __name__ == '__main__':
    main()
